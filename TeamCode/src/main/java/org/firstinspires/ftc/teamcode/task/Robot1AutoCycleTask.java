package org.firstinspires.ftc.teamcode.task;

import static org.firstinspires.ftc.teamcode.auto.AutoBase.AA_NUM_CYCLES;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DELAY_INTAKE_ROTATE_BASE_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DELAY_INTAKE_ROTATE_STEP_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DELAY_PRIOR_DELIVERY_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_DRIVE_BACK_OFFSET;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_DRIVE_PICKUP;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_DRIVE_START;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_INTAKE_DELIVERY_DROP_CYCLE;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_INTAKE_SLIDE_STEP;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DURATION_INTAKE_SLIDE_DOWN_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DURATION_INTAKE_SLIDE_UP_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_DELIVERY;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_INTAKE_UP;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_RETRACT;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.WAIT_PRIOR_DRIVE_TO_PICKUP_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.WAIT_PRIOR_RETRACT_MILLIS;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.objectdetector.PoleDetector;

@Config
public class Robot1AutoCycleTask implements Task {

    public static int FAILED_OFFSET_INCHES = 2;

    private final HDWorldRobotBase robot;
    private final AutoStates autoStates;
    private final int sign;

    private Task currentTask;
    private State currentState;
    private double yOffset;
    private double addToPickupOffset;

    public Robot1AutoCycleTask(HDWorldRobotBase robot, AutoStates autoStates, int sign) {
        this.robot = robot;
        this.autoStates = autoStates;
        this.sign = sign;
        currentTask = null;
        currentState = State.INITIALIZED;
        yOffset = 0.0;
        addToPickupOffset = 0;
    }

    @Override
    public boolean perform() {
        switch (currentState) {
            case INITIALIZED:
                yOffset = (autoStates.getCycleNumber() - 1) * DIST_DRIVE_BACK_OFFSET;
                TrajectorySequenceBuilder forwardSeq =
                        robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
                forwardSeq.lineToLinearHeading(new Pose2d(-DIST_DRIVE_START,
                        -sign * (DIST_DRIVE_PICKUP + yOffset + autoStates.getCurPickupOffset()),
                        -sign * Math.toRadians(90)));
                autoStates.setCurrSeq(forwardSeq.build());
                // moving forwards task:
                currentTask = new ParallelTask(
                        new SeriesTask(
                                new DeliverySlideTask(robot, 0, POWER_RETRACT),
                                new UpdateConeInDeliveryTask(robot, autoStates)),
                        new IntakeSlideTask(robot,
                                (5 - autoStates.getCycleNumber()) * DIST_INTAKE_SLIDE_STEP,
                                POWER_INTAKE_DOWN, DURATION_INTAKE_SLIDE_DOWN_MILLIS),
                        new SeriesTask(
                                new SleepTask(WAIT_PRIOR_DRIVE_TO_PICKUP_MILLIS),
                                new ParallelTask(
                                        new DrivingTask(robot, autoStates.getCurrSeq()),
                                        new SeriesTask(
                                                new WaitForAnyConditionTask(
                                                        new SleepTask(1400),
                                                        new ConditionalTask(() -> robot.isCone() &&
                                                                robot.getConeDistanceInch() < 0.8)),
                                                new CheckConditionalTask(() -> !autoStates.isConeInDelivery() &&
                                                        robot.isCone() && robot.getConeDistanceInch() < 0.8,
                                                        new IntakeClawTask(robot, false))))));
                currentState = State.MOVING_FORWARDS;
                break;
            case MOVING_FORWARDS:
                if (currentTask.perform()) {
                    if (autoStates.isConeInDelivery()) {
                        PoleDetector.AngleAndDist angleAndDist = autoStates.getPoleAngleAndDist();
                        if (angleAndDist == null || angleAndDist.dist > 100) {
                            autoStates.changeCurDeliveryOffset(-2 * FAILED_OFFSET_INCHES * sign);
                        } else if (angleAndDist.angle * sign < -15) {
                            autoStates.changeCurDeliveryOffset(-FAILED_OFFSET_INCHES * sign);
                        } else if (angleAndDist.angle * sign > -2) {
                            autoStates.changeCurDeliveryOffset(FAILED_OFFSET_INCHES * sign);
                        }
                        Log.i("allendebug", angleAndDist == null ? "null" : angleAndDist.angle +
                                "");
                        Log.i("allendebug", "offset " + autoStates.getCurDeliveryOffset());
                        TrajectorySequenceBuilder backSeq =
                                robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
                        backSeq.lineToLinearHeading(new Pose2d(-DIST_DRIVE_START,
                                -sign * yOffset + autoStates.getCurDeliveryOffset(),
                                -sign * Math.toRadians(90)));
                        autoStates.setCurrSeq(backSeq.build());
                        // moving back task:
                        currentTask = new ParallelTask(
                                new SeriesTask(
                                        new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                                        new IntakeSlideTask(robot,
                                                (5 - autoStates.getCycleNumber()) * DIST_INTAKE_SLIDE_STEP,
                                                POWER_INTAKE_DOWN, DURATION_INTAKE_SLIDE_DOWN_MILLIS),
                                        new IntakeClawTask(robot, true)),
                                new SeriesTask(
                                        // wait more because no intake movements:
                                        new SleepTask(DELAY_PRIOR_DELIVERY_MILLIS + 500),
                                        new DeliverySlideTask(robot, robot.getDeliveryHeightHigh(), POWER_DELIVERY),
                                        new SleepTask(WAIT_PRIOR_RETRACT_MILLIS)),
                                new SeriesTask(
                                        new DrivingTask(robot, autoStates.getCurrSeq()),
                                        new PoleDetectionTask(robot, autoStates)));
                        currentState = State.MOVING_BACK_AND_DELIVERING;
                    } else {
                        if (robot.isCone() && robot.getConeDistanceInch() < 0.8) {
                            // moving back task:
                            currentTask = new SeriesTask(
                                    new IntakeClawTask(robot, false),
                                    generateNormalMovingBackTask());
                            currentState = State.MOVING_BACK_AND_DELIVERING;
                        } else {
                            TrajectorySequenceBuilder forwardSeq2 =
                                    robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
                            forwardSeq2.lineToLinearHeading(new Pose2d(-DIST_DRIVE_START,
                                    -sign * (DIST_DRIVE_PICKUP + yOffset + 2 +
                                            autoStates.getCurPickupOffset()),
                                    -sign * Math.toRadians(90)));
                            addToPickupOffset += 1.0;
                            autoStates.setCurrSeq(forwardSeq2.build());
                            // intake first task:
                            currentTask = new ParallelTask(
                                    new IntakeClawTask(robot, IntakeClawTask.State.FULL_OPEN),
                                    new DrivingTask(robot, autoStates.getCurrSeq()));
                            currentState = State.INTAKE_FIRST;
                        }
                    }
                }
                break;
            case INTAKE_FIRST:
                if (currentTask.perform()) {
                    if (robot.isCone() && robot.getConeDistanceInch() < 0.8) {
                        // moving back task:
                        currentTask = new SeriesTask(
                                new IntakeClawTask(robot, false),
                                generateNormalMovingBackTask());
                        currentState = State.MOVING_BACK_AND_DELIVERING;
                    } else {
                        TrajectorySequenceBuilder forwardSeq3 =
                                robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
                        forwardSeq3.lineToLinearHeading(new Pose2d(-DIST_DRIVE_START,
                                -sign * (DIST_DRIVE_PICKUP + yOffset + 4 +
                                        autoStates.getCurPickupOffset()),
                                -sign * Math.toRadians(90)));
                        autoStates.setCurrSeq(forwardSeq3.build());
                        addToPickupOffset += 1.0;
                        // intake second task:
                        currentTask = new ParallelTask(
                                new IntakeClawTask(robot, IntakeClawTask.State.FULL_OPEN),
                                new DrivingTask(robot, autoStates.getCurrSeq()));
                        currentState = State.INTAKE_SECOND;
                    }
                }
                break;
            case INTAKE_SECOND:
                if (currentTask.perform()) {
                    // mitigate false cycle:
                    if (!robot.isCone()) autoStates.setCycleNumber(AA_NUM_CYCLES);
                    // moving back task:
                    currentTask = new SeriesTask(
                            new IntakeClawTask(robot, false),
                            generateNormalMovingBackTask());
                    currentState = State.MOVING_BACK_AND_DELIVERING;
                }
                break;
            case MOVING_BACK_AND_DELIVERING:
                if (currentTask.perform()) {
                    autoStates.changeCurPickupOffset(addToPickupOffset);
                    Log.i("allendebug", "pickup offset " + autoStates.getCurPickupOffset());
                    currentState = State.FINISHED;
                }
                break;
            default:
                break;
        }
        return currentState == State.FINISHED;
    }

    private Task generateNormalMovingBackTask() {
        TrajectorySequenceBuilder backSeq =
                robot.trajectorySequenceBuilder(autoStates.getCurrSeq().end());
        backSeq.lineToLinearHeading(new Pose2d(-DIST_DRIVE_START,
                -sign * yOffset + autoStates.getCurDeliveryOffset(),
                -sign * Math.toRadians(90)));
        autoStates.setCurrSeq(backSeq.build());
        return new ParallelTask(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeSlideTask(robot, robot.getAutoIntakeDeliveryHeightInch(),
                                        POWER_INTAKE_UP, DURATION_INTAKE_SLIDE_UP_MILLIS),
                                new SeriesTask(
                                        new SleepTask(DELAY_INTAKE_ROTATE_BASE_MILLIS +
                                                        (autoStates.getCycleNumber() - 1) *
                                                                DELAY_INTAKE_ROTATE_STEP_MILLIS),
                                        new IntakeRotateTask(robot,
                                                robot.getIntakeDeliveryRotateDegree(),
                                                AngleType.DEGREE))),
                        new ParallelTask(
                                new IntakeSlideTask(robot, robot.getAutoIntakeDeliveryHeightInch() -
                                                DIST_INTAKE_DELIVERY_DROP_CYCLE, 1.0,
                                        DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS),
                                new SeriesTask(
                                        new SleepTask(DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS - 110),
                                        new IntakeClawTask(robot, IntakeClawTask.State.HALF_OPEN))),
                        new IntakeSlideTask(robot,
                                robot.getAutoIntakeDeliveryHeightInch(), 1.0,
                                DURATION_INTAKE_SLIDE_DROP_CYCLE_MILLIS),
                        new Robot1AutoNormalDeliveryTask(robot, autoStates)),
                new SeriesTask(
                        new DrivingTask(robot, autoStates.getCurrSeq()),
                        new PoleDetectionTask(robot, autoStates)));
    }

    private enum State {
        INITIALIZED,
        MOVING_FORWARDS,
        INTAKE_FIRST,
        INTAKE_SECOND,
        MOVING_BACK_AND_DELIVERING,
        FINISHED
    }
}
