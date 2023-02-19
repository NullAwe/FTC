package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;

@Config
@TeleOp(name="Tele-Op - Robot 1", group="competition")
public class TeleOpRobot1 extends TeleOpBase {
    @Override
    protected HDWorldRobotBase createRobot(HardwareMap hardwareMap) {
        return new WorldRobot1(hardwareMap);
    }
}
