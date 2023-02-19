package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// Speed servo with 90 rpm under 4.8v and 270 degree max
public class IntakeRotateTask extends TimedTask {
    private final HDWorldRobotBase robot;
    private final double angleRadian;
    private boolean started = false;

    public IntakeRotateTask(HDWorldRobotBase robot, double angle, AngleType angleType) {
        this(robot, angle, angleType, 300);
    }

    public IntakeRotateTask(HDWorldRobotBase robot, double angle, AngleType angleType,
            int finishTimeMillis) {
        this.robot = robot;
        this.angleRadian = angleType == AngleType.DEGREE ? Math.toRadians(angle) : angle;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setIntakeRotateAngle(angleRadian);
        }
        return false;
    }
}
