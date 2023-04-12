package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

public class UpdateConeInDeliveryTask implements Task {

    private final HDWorldRobotBase robot;
    private final AutoStates autoStates;

    public UpdateConeInDeliveryTask(HDWorldRobotBase robot, AutoStates autoStates) {
        this.robot = robot;
        this.autoStates = autoStates;
    }

    @Override
    public boolean perform() {
        autoStates.setConeInDelivery(robot.deliveryHasCone());
        return true;
    }
}
