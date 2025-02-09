package org.firstinspires.ftc.teamcode.auto.auto_robot2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.DeliverySlideTask;
import org.firstinspires.ftc.teamcode.task.DrivingTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.WaitForAnyConditionTask;

@Config
public abstract class AutoBaseRobot2 extends AutoBase {
    public static double DIST_DRIVE_START = 52;
    public static double DIST_DRIVE_END = 23.5;
    public static double DIST_DRIVE_PICKUP = 25.5;
    // The offset distance when driving backward compared to driving forward to compensate the robot
    // driving characteristic difference between forward and backward
    public static double DIST_DRIVE_BACK_OFFSET = -0.4;
    public static double DIST_INTAKE_SLIDE_STEP = 1.25;
    // The intake slide drop distance before dropping the cone (for the initial start run)
    public static double DIST_INTAKE_DELIVERY_DROP_START = 2;
    // The intake slide drop distance before dropping the cone (for the 5-cycle run)
    public static double DIST_INTAKE_DELIVERY_DROP_CYCLE = 2;

    // Pause time before retracing the delivery slide.
    public static int WAIT_PRIOR_RETRACT_MILLIS = 120;
    public static int WAIT_PRIOR_DRIVE_TO_PICKUP_MILLIS = 5;
    public static int DELAY_PRIOR_DELIVERY_MILLIS = 150;
    public static int DELAY_INTAKE_ROTATE_MILLIS = 80;
    public static int DELAY_INTAKE_ROTATE_STEP_MILLIS = 50;

    // Total time needed for moving the intake slide up to the highest position.
    public static int DURATION_INTAKE_SLIDE_UP_MILLIS = 400;
    // Total time needed for moving the intake slide down to the ready position.
    public static int DURATION_INTAKE_SLIDE_DOWN_MILLIS = 500;
    // Total time needed for moving the intake slide down to ready to drop cone position (for start
    // run).
    public static int DURATION_INTAKE_SLIDE_DROP_START_MILLIS = 300;
    // Total time needed for moving the intake slide down to ready to drop cone position (for
    // 5-cycLe runs).
    public static int DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS = 350;

    public static double POWER_RETRACT = 0.8;
    public static double POWER_DELIVERY = 1.0;
    public static double POWER_INTAKE_UP = 1.0;
    public static double POWER_INTAKE_DOWN = 0.8;
    public static int parkingZone = 3; // 1, 2, 3
    private final ElapsedTime timer = new ElapsedTime();
    protected HDWorldRobotBase robot;
    private AutoState state = AutoState.INIT;
    private int cycleNumber = 0;
    private TrajectorySequence prevSeq;
    private TrajectorySequence currSeq;

    @Override
    protected Task createStartTask() {
        currSeq = robot.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(DIST_DRIVE_START * getSign()).build();
        return new ParallelTask(
                new SeriesTask(
                        new DeliveryRotateTask(robot,
                                robot.getAutoDeliveryRotateAngleDegree() * getSign(),
                                AngleType.DEGREE),
                        new ParallelTask(
                                new IntakeSlideTask(robot, robot.getAutoIntakeDeliveryHeightInch(),
                                        POWER_INTAKE_UP, DURATION_INTAKE_SLIDE_UP_MILLIS),
                                new SeriesTask(
                                        new SleepTask(DELAY_INTAKE_ROTATE_MILLIS +
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
                                        new IntakeClawTask(robot, true)
                                )
                        ),
                        new IntakeSlideTask(robot,
                                robot.getAutoIntakeDeliveryHeightInch(), 1.0,
                                DURATION_INTAKE_SLIDE_DROP_START_MILLIS),
                        getDeliveryTask()
                ),
                new DrivingTask(robot, currSeq)
        );
    }

    @Override
    protected Task getDeliveryTask() {
        return new ParallelTask(
                new SeriesTask(
                        new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                        new IntakeClawTask(robot, false),
                        new IntakeSlideTask(robot, (4 - cycleNumber) * DIST_INTAKE_SLIDE_STEP,
                                POWER_INTAKE_DOWN, DURATION_INTAKE_SLIDE_DOWN_MILLIS),
                        new IntakeClawTask(robot, true)
                ),
                new SeriesTask(
                        new SleepTask(DELAY_PRIOR_DELIVERY_MILLIS),
                        new DeliverySlideTask(robot, robot.getDeliveryHeightHigh(), POWER_DELIVERY),
                        new SleepTask(WAIT_PRIOR_RETRACT_MILLIS)
                )
        );
    }

    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;

        prevSeq = currSeq;
        TrajectorySequenceBuilder finishSeq = robot.trajectorySequenceBuilder(currSeq.end());
        finishSeq.lineToLinearHeading(
                new Pose2d((2 - parkingZone) * DIST_DRIVE_END * getSign() + 2,
                        -DIST_DRIVE_START * getSign(),
                        Math.toRadians(-90)));
        currSeq = finishSeq.build();
        Task drivingTask = new DrivingTask(robot, currSeq, false);

        return new ParallelTask(
                new SeriesTask(
                        new SleepTask(300),
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
        TrajectorySequenceBuilder forwardSeq = robot.trajectorySequenceBuilder(currSeq.end());
        forwardSeq.forward(DIST_DRIVE_PICKUP);
        prevSeq = forwardSeq.build();
        TrajectorySequenceBuilder backSeq = robot.trajectorySequenceBuilder(prevSeq.end());
        backSeq.back(DIST_DRIVE_PICKUP + DIST_DRIVE_BACK_OFFSET);
        currSeq = backSeq.build();

        return new ParallelTask(
                new DeliverySlideTask(robot, 0, POWER_RETRACT),
                new SeriesTask(
                        new SleepTask(WAIT_PRIOR_DRIVE_TO_PICKUP_MILLIS),
                        new ParallelTask(
                                new DrivingTask(robot, prevSeq),
                                new SeriesTask(
                                        new WaitForAnyConditionTask(
                                                new SleepTask(1500),
                                                new ConditionalTask(() -> robot.isCone() &&
                                                        robot.getConeDistanceInch() < 1.5)),
                                        new IntakeClawTask(robot, false))),
                        new ParallelTask(
                                new SeriesTask(
                                        new ParallelTask(
                                                new IntakeSlideTask(robot,
                                                        robot.getAutoIntakeDeliveryHeightInch(),
                                                        POWER_INTAKE_UP,
                                                        DURATION_INTAKE_SLIDE_UP_MILLIS),
                                                new SeriesTask(
                                                        new SleepTask(DELAY_INTAKE_ROTATE_MILLIS +
                                                                (cycleNumber - 1) *
                                                                        DELAY_INTAKE_ROTATE_STEP_MILLIS),
                                                        new IntakeRotateTask(robot,
                                                                robot.getIntakeDeliveryRotateDegree(),
                                                                AngleType.DEGREE)
                                                )
                                        ),
                                        new ParallelTask(
                                                new IntakeSlideTask(robot,
                                                        robot.getAutoIntakeDeliveryHeightInch() -
                                                                DIST_INTAKE_DELIVERY_DROP_CYCLE,
                                                        1.0,
                                                        DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS),
                                                new SeriesTask(
                                                        new SleepTask(
                                                                DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS -
                                                                        110),
                                                        new IntakeClawTask(robot, true)
                                                )
                                        ),
                                        new IntakeSlideTask(robot,
                                                robot.getAutoIntakeDeliveryHeightInch(), 1.0,
                                                DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS),
                                        getDeliveryTask()),
                                new DrivingTask(robot, currSeq)
                        )
                )
        );
    }
}
