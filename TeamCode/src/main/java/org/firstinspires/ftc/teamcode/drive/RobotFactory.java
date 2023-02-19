package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.HDRobotBase.ACTIVE_ROBOT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;

public class RobotFactory {
    public static HDRobotBase createRobot(HardwareMap hardwareMap) {
        return new WorldRobot1(hardwareMap);
//        return ACTIVE_ROBOT == 1 ? new WorldRobot1(hardwareMap) : new WorldRobot1(hardwareMap);
    }
}
