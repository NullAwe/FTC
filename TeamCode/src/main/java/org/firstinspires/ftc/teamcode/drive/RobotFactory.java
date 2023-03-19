package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.HDRobotBase.ACTIVE_ROBOT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.drive.robot2.WorldRobot2;

public class RobotFactory {
    public static HDRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        return ACTIVE_ROBOT == 1 ? new WorldRobot1(hardwareMap, telemetry) :
                new WorldRobot2(hardwareMap, telemetry);
    }
}
