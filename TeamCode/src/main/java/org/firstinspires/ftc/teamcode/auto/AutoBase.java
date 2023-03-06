package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.task.DriveStraightTask;

import kotlin.text.CharDirectionality;

@Config
@TeleOp(name="Auto - Robot 1", group="competition")
public class AutoBase extends LinearOpMode {
    WorldRobot1 robot = new WorldRobot1(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        double dist = 16.0;
        DriveStraightTask task = new DriveStraightTask(robot, dist);
        waitForStart();

        int round = 0;
        while (opModeIsActive() && !task.perform() && round < 10) {
            if (task != null && task.perform()) {
                dist *= -1.0;
                task = new DriveStraightTask(robot, dist);
                round++;
            }
        }
    }
}
