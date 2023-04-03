package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.drive.RobotFactory.createRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
@Autonomous(name="Spline Test", group = "test")
public class SplineTest extends LinearOpMode {
    public static double X1 = -26; // in
    public static double Y1 = 0; // in
    public static double TANGENT1 = 0; // in
    public static double X2 = -52; // in
    public static double Y2 = 0; // in
    public static double TANGENT2 = 90; // in


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        HDRobotBase drive = createRobot(hardwareMap, telemetry);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(X1, Y1, Math.toRadians(TANGENT1)))
                .lineToLinearHeading(new Pose2d(X2, Y2, Math.toRadians(TANGENT2)))
                .build();
        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("total time", (int) timer.milliseconds());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
