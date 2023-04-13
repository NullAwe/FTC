package org.firstinspires.ftc.teamcode.auto.auto_robot1_exp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.DeliverySlideTask;
import org.firstinspires.ftc.teamcode.task.DrivingTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.Robot1AutoCycleTask;
import org.firstinspires.ftc.teamcode.task.Robot1AutoNormalDeliveryTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBaseRobot1Exp extends AutoBase {
    public static double DIST_DRIVE_START = 50;
    public static double DIST_DRIVE_END = 23.5;
    // The intake slide drop distance before dropping the cone (for the initial start run)
    public static double DIST_INTAKE_DELIVERY_DROP_START = 3;

    // Pause time before retracing the delivery slide.
    public static int DELAY_INTAKE_ROTATE_BASE_MILLIS = 140;
    public static int DELAY_INTAKE_ROTATE_STEP_MILLIS = 40;
    // Total time needed for moving the intake slide up to the highest position.
    public static int DURATION_INTAKE_SLIDE_UP_MILLIS = 500;
    // Total time needed for moving the intake slide down to ready to drop cone position.
    public static int DURATION_INTAKE_SLIDE_DROP_START_MILLIS = 300;

    public static double POWER_RETRACT = 0.8;
    public static double POWER_INTAKE_UP = 1.0;

    @Override
    protected Task createStartTask() {
        autoStates.setCurrSeq(robot.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-DIST_DRIVE_START, 0, 0))
                .turn(-getSign() * Math.toRadians(90)).build());
        return new ParallelTask(
                new SeriesTask(
                        new DeliveryRotateTask(robot,
                                robot.getAutoDeliveryRotateAngleDegree() * getSign(),
                                AngleType.DEGREE),
                        new ParallelTask(
                                new IntakeSlideTask(robot, robot.getAutoIntakeDeliveryHeightInch(),
                                        POWER_INTAKE_UP, DURATION_INTAKE_SLIDE_UP_MILLIS),
                                new SeriesTask(
                                        new SleepTask(DELAY_INTAKE_ROTATE_BASE_MILLIS +
                                                DELAY_INTAKE_ROTATE_STEP_MILLIS * 5),
                                        new IntakeRotateTask(robot,
                                                robot.getIntakeDeliveryRotateDegree(),
                                                AngleType.DEGREE)
                                )
                        ),
                        new ParallelTask(
                                new IntakeSlideTask(robot,
                                        robot.getAutoIntakeDeliveryHeightInch() -
                                                DIST_INTAKE_DELIVERY_DROP_START,
                                        1.0,
                                        DURATION_INTAKE_SLIDE_DROP_START_MILLIS),
                                new SeriesTask(
                                        new SleepTask(
                                                DURATION_INTAKE_SLIDE_DROP_START_MILLIS - 110),
                                        new IntakeClawTask(robot, IntakeClawTask.State.HALF_OPEN)
                                )
                        ),
                        new IntakeSlideTask(robot,
                                robot.getAutoIntakeDeliveryHeightInch(), 1.0,
                                DURATION_INTAKE_SLIDE_DROP_START_MILLIS),
                        new SleepTask(1500),
                        getDeliveryTask()
                ),
                new DrivingTask(robot, autoStates.getCurrSeq())
        );
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
            finishSeq.turn(Math.toRadians(90 * getSign()));
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
