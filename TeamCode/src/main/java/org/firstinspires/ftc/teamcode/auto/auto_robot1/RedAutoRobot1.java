package org.firstinspires.ftc.teamcode.auto.auto_robot1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;

@Autonomous(name="Auto - Robot 1 - Red", group="Auto1Exp")
public class RedAutoRobot1 extends AutoBaseRobot1 {
    @Override
    protected boolean isBlueCorner() {
        return false;
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
