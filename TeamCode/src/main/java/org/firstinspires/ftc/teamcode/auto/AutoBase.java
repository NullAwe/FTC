package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.task.DriveStraightTask;

import kotlin.text.CharDirectionality;

@Config
@Autonomous(name="Auto - Robot 1", group="competition")
public class AutoBase extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WorldRobot1 robot = new WorldRobot1(hardwareMap);
        double dist = 16.0;
        DriveStraightTask task = new DriveStraightTask(robot, dist);

        waitForStart();

        int round = 0;
        while (opModeIsActive() && round < 10) {
            if (task != null && task.perform()) {
                dist *= -1.0;
                task = new DriveStraightTask(robot, dist);
                round++;
            }
        }
    }
}
