package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.drive.RobotFactory.createRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDRobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(name="Back and Forth Test", group = "test")
public class BackAndForthTest extends LinearOpMode {
    public static double DISTANCE = 26; // in
    public static double L_DISTANCE = 52; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        HDRobotBase drive = createRobot(hardwareMap, telemetry);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(L_DISTANCE)
                .forward(DISTANCE)
                .back(DISTANCE)
                .forward(DISTANCE)
                .back(DISTANCE)
                .forward(DISTANCE)
                .back(DISTANCE)
                .forward(DISTANCE)
                .back(DISTANCE)
                .forward(DISTANCE)
                .back(DISTANCE)
                .build();
        drive.followTrajectorySequence(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("total time", (int) timer.milliseconds());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
