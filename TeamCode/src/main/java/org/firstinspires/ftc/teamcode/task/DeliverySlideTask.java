package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// GoBilda motor 1150rpm with pulley 100mm per resolution
public class DeliverySlideTask extends TimedTask {
    private final HDWorldRobotBase robot;
    private final double targetHeight;
    private final double power;
    private boolean started = false;

    public DeliverySlideTask(HDWorldRobotBase robot, double height) {
        this(robot, height, 0.8);
    }

    public DeliverySlideTask(HDWorldRobotBase robot, double height, double power) {
        this(robot, height, power, 700);
    }
    public DeliverySlideTask(HDWorldRobotBase robot, double height, double power,
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
            robot.setDeliverySlideHeight(targetHeight);
            robot.setDeliverySlidePower(power);
        }
        return false;
    }
}
