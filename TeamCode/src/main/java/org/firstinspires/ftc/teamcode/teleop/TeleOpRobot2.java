package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.drive.robot2.WorldRobot2;

@Config
@TeleOp(name="Tele-Op - Robot 2", group="competition")
public class TeleOpRobot2 extends TeleOpBase {
    public static double INTAKE_HEIGHT_INCHES = 19;
    public static double INTAKE_ROTATE_DROP_DEGREE = -96;
    public static double DELIVERY_HEIGHT_HIGH = 22;
    public static double DELIVERY_HEIGHT_MID = 10;

    @Override
    protected HDWorldRobotBase createRobot(HardwareMap hardwareMap) {
        return new WorldRobot2(hardwareMap);
    }

    @Override
    protected double getIntakeDeliveryHeightInch() {
        return INTAKE_HEIGHT_INCHES;
    }

    @Override
    protected double getIntakeDeliveryRotateDegree() {
        return INTAKE_ROTATE_DROP_DEGREE;
    }

    @Override
    protected double getDeliveryHeightHigh() {
        return DELIVERY_HEIGHT_HIGH;
    }

    @Override
    protected double getDeliveryHeightMedium() {
        return DELIVERY_HEIGHT_MID;
    }
}
