package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// 5-turn servo 90/115rpm at 4.8/6.0V with 1:5 gear ratio
public class DeliveryRotateTask extends TimedTask {
    private final HDWorldRobotBase robot;
    private final double angleRadian;
    private boolean started = false;

    public DeliveryRotateTask(HDWorldRobotBase robot, double angle, AngleType angleType) {
        this(robot, angle, angleType, 200);
    }

    public DeliveryRotateTask(HDWorldRobotBase robot, double angle, AngleType angleType,
            int finishTimeMillis) {
        this.robot = robot;
        this.angleRadian = angleType == AngleType.DEGREE ? Math.toRadians(angle) : angle;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setDeliveryRotateAngle(angleRadian);
        }
        return false;
    }
}
