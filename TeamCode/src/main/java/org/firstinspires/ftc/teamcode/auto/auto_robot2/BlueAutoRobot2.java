package org.firstinspires.ftc.teamcode.auto.auto_robot2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot2.WorldRobot2;

@Autonomous(name="Auto - Robot 2 - Blue", group="Auto2")
public class BlueAutoRobot2 extends AutoBaseRobot2 {
    @Override
    protected boolean isBlueCorner() {
        return true;
    }

    @Override
    protected String getTeleOpName() {
        return "Tele-Op - Robot 2";
    }

    @Override
    protected HDWorldRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        return new WorldRobot2(hardwareMap, telemetry);
    }
}
