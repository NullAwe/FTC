package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.util.objectdetector.ConeDetector;

@Config
public class ConeStackAlignTask implements Task {
    private final ElapsedTime timer = new ElapsedTime();
    private final ConeDetector coneDetector;
    private Double coneAngle = null;
    private TaskState state;
    private boolean cancelRequested = false;

    public ConeStackAlignTask(HDWorldRobotBase robot) {
        this.coneDetector = robot.getConeDetector();
        state = TaskState.PENDING;
    }

    @Override
    public boolean perform() {
        if (coneDetector == null || cancelRequested) {
            state = TaskState.DONE;
        }

        switch (state) {
            case PENDING:
                state = TaskState.CONE_DETECTING;
                timer.reset();
                new Thread(() -> coneAngle = coneDetector.getConeAngle()).start();
                break;
            case CONE_DETECTING:
                if (coneAngle != null) {
                    RobotLog.ee("ConeCone", "camera angle: %f, detecting time %d",
                            Math.toDegrees(coneAngle),
                            (int) timer.milliseconds());
                    timer.reset();
                }
                break;
            default:
                break;
        }
        return state == TaskState.DONE;
    }

    @Override
    public void cancel() {
        if (state == TaskState.CONE_DETECTING) {
            coneDetector.cancel();
        }
        cancelRequested = true;
    }

    enum TaskState {
        PENDING, // task created, but has not performed yet
        CONE_DETECTING,
        DONE
    }
}
