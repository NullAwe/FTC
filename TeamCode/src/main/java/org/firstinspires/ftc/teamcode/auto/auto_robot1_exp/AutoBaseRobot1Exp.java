package org.firstinspires.ftc.teamcode.auto.auto_robot1_exp;

import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DELAY_PRIOR_DELIVERY_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_INTAKE_SLIDE_STEP;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DURATION_INTAKE_SLIDE_DOWN_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_DELIVERY;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.WAIT_PRIOR_RETRACT_MILLIS;

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
import org.firstinspires.ftc.teamcode.task.Robot1AutoCycleTask;
import org.firstinspires.ftc.teamcode.task.Robot1AutoNormalDeliveryTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBaseRobot1Exp extends AutoBase {

    public static double DIST_DRIVE_START = 50;
    public static double DIST_DRIVE_END = 23.5;

    // Total time needed for moving the intake slide down to ready to drop cone position.
    public static int DURATION_INTAKE_SLIDE_DROP_START_MILLIS = 300;

    public static double POWER_RETRACT = 0.8;
    public static double POWER_INTAKE_UP = 1.0;
    public static int PRELOAD_DELIVERY_DELAY_MILLIS = 2000;

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
                new DrivingTask(robot, autoStates.getCurrSeq()));
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
