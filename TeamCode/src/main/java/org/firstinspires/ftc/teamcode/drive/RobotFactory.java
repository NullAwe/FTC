package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.HDRobotBase.ACTIVE_ROBOT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.drive.robot2.WorldRobot2;

public class RobotFactory {
    public static HDRobotBase createRobot(HardwareMap hardwareMap) {
        return ACTIVE_ROBOT == 1 ? new WorldRobot1(hardwareMap) : new WorldRobot2(hardwareMap);
    }
}
