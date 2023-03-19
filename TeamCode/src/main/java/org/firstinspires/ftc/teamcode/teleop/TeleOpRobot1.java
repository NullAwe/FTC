package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;

@Config
@TeleOp(name="Tele-Op - Robot 1", group="competition")
public class TeleOpRobot1 extends TeleOpBase {
    public static double INTAKE_HEIGHT_INCHES = 17;
    public static double INTAKE_ROTATE_DROP_DEGREE = -96;
    public static double DELIVERY_HEIGHT_HIGH = 31;
    public static double DELIVERY_HEIGHT_MID = 14;


    @Override
    protected HDWorldRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        return new WorldRobot1(hardwareMap, telemetry);
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
