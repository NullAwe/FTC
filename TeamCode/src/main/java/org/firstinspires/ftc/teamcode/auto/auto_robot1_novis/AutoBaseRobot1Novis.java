package org.firstinspires.ftc.teamcode.auto.auto_robot1_novis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.DeliverySlideTask;
import org.firstinspires.ftc.teamcode.task.DrivingTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.PoleDetectionTask;
import org.firstinspires.ftc.teamcode.task.Robot1AutoCycleTask;
import org.firstinspires.ftc.teamcode.task.Robot1AutoNormalDeliveryTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBaseRobot1Novis extends AutoBase {

    public static double DIST_DRIVE_START = 51.5;
    public static double DIST_DRIVE_END = 23.5;
    public static double DIST_DRIVE_PICKUP = 26;
    // The offset distance when driving backward compared to driving forward to compensate the robot
    // driving characteristic difference between forward and backward
    public static double DIST_DRIVE_BACK_OFFSET = 0.3;
    public static double DIST_INTAKE_SLIDE_STEP = 1.25;
    // The intake slide drop distance before dropping the cone (for the 5-cycle run)
    public static double DIST_INTAKE_DELIVERY_DROP_CYCLE = 3;

    public static double CYCLE_OFFSET_X = 0.3;

    // Pause time before retracing the delivery slide.
    public static int WAIT_PRIOR_RETRACT_MILLIS = 120;
    public static int WAIT_PRIOR_DRIVE_TO_PICKUP_MILLIS = 250;
    public static int DELAY_PRIOR_DELIVERY_MILLIS = 100;
    public static int DELAY_INTAKE_ROTATE_BASE_MILLIS = 140;
    public static int DELAY_INTAKE_ROTATE_STEP_MILLIS = 40;
    // Total time needed for moving the intake slide up to the highest position.
    public static int DURATION_INTAKE_SLIDE_UP_MILLIS = 500;
    // Total time needed for moving the intake slide down to the
    // ready position.
    public static int DURATION_INTAKE_SLIDE_DOWN_MILLIS = 500;
    // Total time needed for moving the intake slide down to ready to drop cone position.
    public static int DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS = 200;
    // Time for robot to deliver before moving:
    public static int PRELOAD_DELIVERY_DELAY_MILLIS = 2000;

    public static double POWER_RETRACT = 0.8;
    public static double POWER_DELIVERY = 1.0;
    public static double POWER_INTAKE_UP = 1.0;
    public static double POWER_INTAKE_DOWN = 0.8;

    @Override
    protected Task createStartTask() {
        autoStates.setCurrSeq(robot.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-DIST_DRIVE_START, 0, 0))
                .turn(-getSign() * Math.toRadians(90), 3 * WorldRobot1.MAX_ANG_VEL,
                        1.5 * WorldRobot1.MAX_ANG_ACCEL).build());
        return new ParallelTask(
                new SeriesTask(
                        new DeliveryRotateTask(robot,
                                robot.getAutoDeliveryRotateAngleDegree() * getSign(),
                                AngleType.DEGREE),
                        new ParallelTask(
                                new IntakeSlideTask(robot,
                                        (5 - autoStates.getCycleNumber()) * DIST_INTAKE_SLIDE_STEP,
                                        POWER_INTAKE_DOWN, DURATION_INTAKE_SLIDE_DOWN_MILLIS),
                                new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                                new IntakeClawTask(robot, true)),
                        new SleepTask(PRELOAD_DELIVERY_DELAY_MILLIS),
                        new DeliverySlideTask(robot, robot.getDeliveryHeightHigh(), POWER_DELIVERY),
                        new SleepTask(WAIT_PRIOR_RETRACT_MILLIS)),
                new SeriesTask(
                        new DrivingTask(robot, autoStates.getCurrSeq()),
                        new PoleDetectionTask(robot, autoStates)));
    }

    @Override
    protected Task getDeliveryTask() {
        return new Robot1AutoNormalDeliveryTask(robot, autoStates);
    }

    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        TrajectorySequenceBuilder finishSeq = robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
        if (parkingZone == 2) {
            finishSeq.turn(Math.toRadians(90 * getSign()), 4 * WorldRobot1.MAX_ANG_VEL,
                    2 * WorldRobot1.MAX_ANG_ACCEL);
        } else {
            double pos = (parkingZone - 2) * DIST_DRIVE_END;
            if (parkingZone == 1 && !isBlueCorner()) pos -= 2;
            else if (parkingZone == 3 && isBlueCorner()) pos += 2;
            finishSeq.lineToLinearHeading(
                    new Pose2d(-DIST_DRIVE_START, pos, Math.toRadians(0)));
        }
        autoStates.setCurrSeq(finishSeq.build());

        Task drivingTask = new DrivingTask(robot, autoStates.getCurrSeq(), false);

        return new ParallelTask(
                new SeriesTask(
                        new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                        new ParallelTask(
                                new IntakeSlideTask(robot, 0),
                                new DeliveryRotateTask(robot, 0, AngleType.DEGREE),
                                drivingTask)
                ),
                new DeliverySlideTask(robot, 0, POWER_RETRACT));
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        return new Robot1AutoCycleTask(robot, autoStates, getSign());
    }
}
