package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.util.objectdetector.PoleDetector;

public class PoleDetectionTask implements Task {

    enum TaskState {
        PENDING, // task created, but has not performed yet
        POLE_DETECTING,
        DONE
    }

    private final PoleDetector poleDetector;
    private final AutoStates autoStates;
    private boolean cancelRequested = false;

    private TaskState state;

    private final ElapsedTime timer = new ElapsedTime();

    public PoleDetectionTask(HDWorldRobotBase robot, AutoStates autoStates) {
        this.poleDetector = robot.getPoleDetector();
        this.autoStates = autoStates;
        state = TaskState.PENDING;
    }

    @Override
    public boolean perform() {
        if (poleDetector == null || cancelRequested) {
            state = TaskState.DONE;
        }

        switch (state) {
            case PENDING:
                state = TaskState.POLE_DETECTING;
                timer.reset();
                autoStates.setPoleAngleAndDist(null);
                new Thread(() -> autoStates.setPoleAngleAndDist(poleDetector.getPoleAngleAndDist())).start();
                break;
            case POLE_DETECTING:
                if (autoStates.getPoleAngleAndDist() != null) {
                    RobotLog.ee("PolePole", "camera angle: %f, dist: %f, detecting time %d",
                            Math.toDegrees(autoStates.getPoleAngleAndDist().angle),
                            autoStates.getPoleAngleAndDist().dist, (int) timer.milliseconds());
                    state = TaskState.DONE;
                }
                break;
            default:
                // Must be in the DONE state
                break;
        }

        return state == TaskState.DONE;
    }

    @Override
    public void cancel() {
        if (state == TaskState.POLE_DETECTING) {
            poleDetector.cancel();
        }
        cancelRequested = true;
    }
}
