package org.firstinspires.ftc.teamcode.auto_robot1_test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;

@Config
@Autonomous(name="Auto - Robot 1 - Blue - experiment", group="world")
public class BlueAutoRobot1Exp extends AutoBase1Exp {
    @Override
    protected boolean isBlueCorner() {
        return true;
    }

    @Override
    protected String getTeleOpName() {
        return "Tele-Op - Robot 1";
    }

    @Override
    protected HDWorldRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        return new WorldRobot1(hardwareMap, telemetry);
    }
}
