package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// 5-turn servo 90/115rpm at 4.8/6.0V with 1:5 gear ratio
public class ConeRighterTask extends TimedTask {
    public enum Position {
        UP,
        DOWN
    }

    // Total time required to turn 90 degree for a GoBilda super speed servo.
    private final static int FINISH_TIME_MILLIS = 85;
    private final HDWorldRobotBase robot;
    private final Position pos;
    private boolean started = false;

    public ConeRighterTask(HDWorldRobotBase robot, Position pos) {
        this.robot = robot;
        this.pos = pos;
        setFinishTimeMillis(FINISH_TIME_MILLIS);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            if (robot.isConeRighterUp() == (pos == Position.UP))
                return true;
            robot.toggleConeRighter();
        }
        return false;
    }
}
