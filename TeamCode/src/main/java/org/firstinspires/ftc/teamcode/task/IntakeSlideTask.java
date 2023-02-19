package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

public class IntakeSlideTask extends TimedTask {
    private final HDWorldRobotBase robot;
    private final double targetHeight;
    private final double power;
    private boolean started = false;

    public IntakeSlideTask(HDWorldRobotBase robot, double height) {
        this(robot, height, 0.8);
    }

    public IntakeSlideTask(HDWorldRobotBase robot, double height, double power) {
        this(robot, height, power, 400);
    }
    public IntakeSlideTask(HDWorldRobotBase robot, double height, double power,
            int finishTimeMillis) {
        this.robot = robot;
        this.targetHeight = height;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setIntakeSlideHeight(targetHeight);
            robot.setIntakeSlidePower(power);
        }
        return false;
    }
}
