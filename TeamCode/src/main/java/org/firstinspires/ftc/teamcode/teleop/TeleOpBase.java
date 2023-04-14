package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.RoadRunnerParameters;
import org.firstinspires.ftc.teamcode.task.DeliverySlideTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;

/**
 * TeleOp Base Class with complete different functionalities.
 */
@Config
public abstract class TeleOpBase extends LinearOpMode {
    // A distance indicating that a cone is close enough to pickup. Any distance larger than that is
    // not safe to grab.
    private static final double MAX_CONE_GRABBING_DIST = 1.0;
    // Min joystick value to start moving or rotating the arm
    private static final double MIN_JOYSTICK_VALUE = 0.001;
    // To avoid closing the claw immediately after opening the claw, we need a gap time.
    private static final int MIN_GAP_MILLIS = 1000;

    // Constants for control driving experience
    public static double ANGULAR_VEL_FACTOR = 0.7;
    public static int RAMP_UP_TIME_MILLIS = 600;
    public static int RAMP_DOWN_TIME_MILLIS = 19;
    public static double INTAKE_SLIDER_POWER = 1.0;
    public static double DELIVERY_POWER = 1.0;
    public static double RETRACT_POWER = 1.0;

    // Testing
    public static int INTAKE_ROTATE_DELAY_MILLIS = 400;
    public static int INTAKE_DELIVERY_PAUSE_MILLIS = 40;
    public static int DELIVERY_DELAY_MILLIS = 300;
    public static double DELIVERY_ANGLE_ROTATE_DEGREE = 65;
    public static int INTAKE_DROP_DELAY = 200;
    private static int INTAKE_SLIDE_HEIGHT_MIN = -1;
    private static int DELIVERY_SLIDE_HEIGHT_MIN = -1;
    protected final ElapsedTime cycleTime = new ElapsedTime();
    private final FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    private final ElapsedTime releaseTime = new ElapsedTime();
    protected HDWorldRobotBase robot;
    //
    GamePad gp1, gp2;
    private Pose2d prevPower = new Pose2d(0.0, 0.0, 0.0);
    private Task currentTask = null;
    private Task resetIntakeTask = null;

    private boolean debugMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        robot = createRobot(hardwareMap, telemetry);

        robot.setIntakeRotateAngle(0.0);
        robot.openClaw();
        robot.setDeliveryRotateAngle(0);
        robot.initConeRighter();

        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        waitForStart();

        cycleTime.reset();
        while (opModeIsActive()) {
            // Call this every cycle to cache the encoder position and velocity.
            robot.updateEncoderValues();

            if (!debugMode) {
                driveRobot(gp1);
                if (gp1.onceA()) {
                    robot.toggleConeRighter();
                }
                if (resetIntakeTask != null) {
                    if (gp1.onceX()) {
                        resetIntakeTask.cancel();
                        resetIntakeTask = null;
                    } else if (resetIntakeTask.perform()) {
                        resetIntakeTask = null;
                    }
                }

                if (currentTask != null) {
                    if (gp1.onceX()) {
                        currentTask.cancel();
                        currentTask = null;
                    } else if (currentTask.perform()) {
                        currentTask = null;
                    }
                } else {
                    if (gp2.onceY() || gp2.rightBumper() && gp2.leftBumper()) {
                        currentTask = new DeliverySlideTask(robot, 0, RETRACT_POWER);
                    } else {
                        if (gp2.leftTrigger() > 0.1 && gp2.rightTrigger() > 0.1) {
                            robot.setDeliveryRotateAngle(0);
                        } else if (gp2.onceLeftTrigger()) {
                            robot.setDeliveryRotateAngle(
                                    Math.toRadians(DELIVERY_ANGLE_ROTATE_DEGREE));
                        } else if (gp2.onceRightTrigger()) {
                            robot.setDeliveryRotateAngle(
                                    -Math.toRadians(DELIVERY_ANGLE_ROTATE_DEGREE));
                        }
                    }
                    if (resetIntakeTask == null) {
                        double intakeHeight = robot.getIntakeSlidePositionInches();
                        if (gp2.onceX()) {
                            // Open or close claw/gripper
                            robot.toggleClaw();
                            releaseTime.reset();
                        } else if (gp2.onceB() && !robot.isClawOpen()) {
                            currentTask = getPickUpConeTask();
                        } else if (gp2.onceA()) {
                            resetIntakeTask = getResetIntakeTask();
                        } else if (Math.abs(gp2.rightStickY()) > 0.1 &&
                                Math.abs(gp2.rightStickY()) > Math.abs(gp2.rightStickX())) {
                            intakeHeight -= gp2.rightStickY() / 0.2;
                            intakeHeight = Math.max(Math.min(intakeHeight, getIntakeHeightMax()),
                                    INTAKE_SLIDE_HEIGHT_MIN);
                            currentTask =
                                    new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
                        } else if (gp2.onceDpadUp()) {
                            intakeHeight += 1.25;
                            intakeHeight = Math.max(Math.min(intakeHeight, getIntakeHeightMax()),
                                    INTAKE_SLIDE_HEIGHT_MIN);
                            currentTask =
                                    new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
                        } else if (gp2.onceDpadDown()) {
                            intakeHeight -= 1.25;
                            intakeHeight = Math.max(Math.min(intakeHeight, getIntakeHeightMax()),
                                    INTAKE_SLIDE_HEIGHT_MIN);
                            currentTask =
                                    new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
                        } else if (Math.abs(gp2.rightStickX()) > 0.1) {
                            robot.setIntakeRotateAngle(
                                    robot.getIntakeRotateAngle() + gp2.rightStickX() / 50);
                        } else if (gp2.onceRightBumper() && readyToDeliverCone()) {
                            currentTask = new SeriesTask(
                                    new SleepTask(DELIVERY_DELAY_MILLIS),
                                    new DeliverySlideTask(robot, robot.getDeliveryHeightHigh(),
                                            DELIVERY_POWER));
                            resetIntakeTask = getResetIntakeTask();
                        } else if (gp2.onceLeftBumper() && readyToDeliverCone()) {
                            currentTask = new SeriesTask(
                                    new SleepTask(DELIVERY_DELAY_MILLIS),
                                    new DeliverySlideTask(robot, robot.getDeliveryHeightMedium(),
                                            DELIVERY_POWER));
                            resetIntakeTask = getResetIntakeTask();
                        } else if (Math.abs(gp2.leftStickY()) > 0.1) {
                            double deliveryHeight = robot.getDeliverySlidePositionInches();
                            deliveryHeight -= gp2.leftStickY() / 0.2;
                            deliveryHeight =
                                    Math.max(Math.min(deliveryHeight, getDeliveryHeightMax()),
                                            DELIVERY_SLIDE_HEIGHT_MIN);
                            currentTask =
                                    new DeliverySlideTask(robot, deliveryHeight, DELIVERY_POWER);
                        } else if (gp2.onceStart()) {
                            currentTask = new IntakeSlideTask(robot,
                                    robot.getIntakeHeightLowJunction());
                        } else if (robot.isCone() &&
                                robot.getConeDistanceInch() < MAX_CONE_GRABBING_DIST &&
                                robot.isClawOpen() && releaseTime.milliseconds() > MIN_GAP_MILLIS) {
                            currentTask = getSecureConeTask();
                        }
                    }
                }
            } else {
                if (currentTask != null && (currentTask.perform() || gp2.onceX())) {
                    currentTask.cancel();
                    currentTask = null;
                }
                robot.setDeliverySlidePowerWithoutEncoder(-gp2.leftStickY() * 0.6);
                robot.setIntakeSlidePowerWithoutEncoder(gp2.rightStickY() * 0.6);
            }

            if (gp2.back() && gp2.onceX()) {
                if (currentTask != null) currentTask.cancel();
                currentTask = null;
                if (resetIntakeTask != null) resetIntakeTask.cancel();
                resetIntakeTask = null;
                debugMode = !debugMode;
                if (!debugMode) {
                    robot.restartMotors();
                }
            }

            // Test:
            telemetry.update();

            gp1.update();
            gp2.update();
            if (isStopRequested()) {
                try {
                    robot.stopFrontCameraSource();
                    RobotLog.ee("TeleOpBase", "Shutting down the camera stream");
                } catch (Exception e) {
                    RobotLog.ee("TeleOpBase", e, "failed to shut down camera");
                }
            }
        }
    }

    private Task getPickUpConeTask() {
        return new ParallelTask(
                new IntakeSlideTask(robot, robot.getTeleopIntakeDeliveryHeightInch(),
                        INTAKE_SLIDER_POWER),
                new SeriesTask(
                        new SleepTask(INTAKE_ROTATE_DELAY_MILLIS),
                        new IntakeRotateTask(robot, robot.getIntakeDeliveryRotateDegree(),
                                AngleType.DEGREE),
                        new IntakeSlideTask(robot, robot.getTeleopIntakeDeliveryHeightInch() - 1,
                                INTAKE_SLIDER_POWER, INTAKE_DROP_DELAY)));
    }

    private Task getSecureConeTask() {
        return new SeriesTask(
                new IntakeClawTask(robot, /*open=*/false),
                new IntakeSlideTask(robot, robot.getIntakeSlidePositionInches() + 4,
                        INTAKE_SLIDER_POWER, 100));
    }

    private boolean readyToDeliverCone() {
        if (!robot.isCone()) return true;

        double diffToTargetHeight =
                robot.getIntakeSlidePositionInches() - robot.getTeleopIntakeDeliveryHeightInch();
        double diffToTargetAngle = robot.getIntakeDeliveryRotateDegree() -
                Math.toDegrees(robot.getIntakeRotateAngle());
        return Math.abs(diffToTargetAngle) < 5 && diffToTargetHeight > -2;
    }

    private Task getResetIntakeTask() {
        if (robot.getIntakeSlidePositionInches() < 0.1) return null;

        boolean hasCone = robot.isCone();
        double diffToTargetAngle = robot.getIntakeDeliveryRotateDegree() -
                Math.toDegrees(robot.getIntakeRotateAngle());
        SeriesTask task = new SeriesTask();
        if (diffToTargetAngle < -5) {  // Intake is not above the cone holder
            task.add(new IntakeRotateTask(robot, 0, AngleType.DEGREE));
            if (robot.isClawOpen()) {
                task.add(new IntakeClawTask(robot, IntakeClawTask.State.HALF_OPEN));
            }
            task.add(new IntakeSlideTask(robot, 0, INTAKE_SLIDER_POWER));
            if (!hasCone) {
                task.add(new IntakeClawTask(robot, true));
            }
        } else {
            if (robot.isCone()) {
                task.add(new IntakeClawTask(robot, IntakeClawTask.State.HALF_OPEN));
                task.add(new IntakeSlideTask(robot, getIntakeHeightMax(), 1.0,
                        robot.getIntakeSlideDropUpDelay()));
            }
            task.add(new IntakeRotateTask(robot, 0, AngleType.DEGREE));
            if (robot.isClawOpen()) {
                task.add(new IntakeClawTask(robot, IntakeClawTask.State.HALF_OPEN));
            }
            task.add(new IntakeSlideTask(robot, 0, INTAKE_SLIDER_POWER));
            task.add(new IntakeClawTask(robot, IntakeClawTask.State.FULL_OPEN));
        }
        return task;
    }

    private void driveRobot(GamePad gp1) {
        if (gp1.dpadUp() || gp1.dpadDown() || gp1.dpadLeft() || gp1.dpadRight()) {
            double power = 0.4 * (gp1.leftBumper() ? 1.5 : 1.0);
            double multiplierX = gp1.dpadDown() ? -1.0 : (gp1.dpadUp() ? 1.0 : 0);
            double multiplierY = gp1.dpadRight() ? -1.0 : (gp1.dpadLeft() ? 1.0 : 0);
            computePowerWithRampUp(power * multiplierX, power * multiplierY, 0.0);
        } else {
            boolean isX = Math.abs(gp1.leftStickY()) > Math.abs(gp1.leftStickX());
            computePowerWithRampUp(isX ? -gp1.leftStickY() : 0.0, isX ? 0.0 : -gp1.leftStickX(),
                    -gp1.rightStickX() * 0.5);
        }

        RoadRunnerParameters params = robot.getRoadRunnerParameters();
        double angularVel = params.maxRPM / 60 * params.ticksPerRev * ANGULAR_VEL_FACTOR;
        double v = prevPower.getX() - params.lateralMultiplier * prevPower.getY() -
                prevPower.getHeading();
        double v1 = prevPower.getX() + params.lateralMultiplier * prevPower.getY() -
                prevPower.getHeading();
        double v2 = prevPower.getX() - params.lateralMultiplier * prevPower.getY() +
                prevPower.getHeading();
        double v3 = prevPower.getX() + params.lateralMultiplier * prevPower.getY() +
                prevPower.getHeading();
        robot.setVelocities(v * angularVel, v1 * angularVel, v2 * angularVel, v3 * angularVel);
    }

    private void computePowerWithRampUp(double powerX, double powerY, double powerH) {
        double dt = cycleTime.milliseconds();
        cycleTime.reset();
//        double rampUp = RAMP_UP_TIME_MILLIS;
//        double rampDown = RAMP_DOWN_TIME_MILLIS;
//        if (Math.abs(powerX) <= 0.1) {
//            rampUp *= 0.5;
//            rampDown *= 0.5;
//        }
        double maxPowerInc = dt / RAMP_UP_TIME_MILLIS;
        double maxPowerDec = -dt / RAMP_DOWN_TIME_MILLIS;
        prevPower = new Pose2d(
                adjustPower(powerX, prevPower.getX(), maxPowerInc, maxPowerDec),
                adjustPower(powerY, prevPower.getY(), maxPowerInc, maxPowerDec),
                adjustPower(powerH, prevPower.getHeading(), maxPowerInc, maxPowerDec));
    }

    private double adjustPower(double power, double prevP, double maxPowerInc, double maxPowerDec) {
        double powerChangeX = power - prevP;
        if (power > 0 && prevP > -0.0001) {
            if (powerChangeX > maxPowerInc) {
                power = prevP + maxPowerInc;
            } else if (powerChangeX < maxPowerDec) {
                power = prevP + maxPowerDec;
            }
        } else if (power < 0 && prevP < 0.0001) {
            if (-powerChangeX > maxPowerInc) {
                power = prevP - maxPowerInc;
            } else if (-powerChangeX < maxPowerDec) {
                power = prevP - maxPowerDec;
            }
        } else {
            powerChangeX = -prevP;
            if (powerChangeX > 0 && -powerChangeX < maxPowerDec) {
                power = Math.min(prevP - maxPowerDec, 0.0);
            } else if (powerChangeX < 0 && powerChangeX < maxPowerDec) {
                power = Math.max(0, prevP + maxPowerDec);
            }
        }
        return power;
    }

    private double getIntakeHeightMax() {
        return robot.getIntakeHeightMax();
    }

    private double getDeliveryHeightMax() {
        return robot.getDeliveryHeightHigh() + 3;
    }

    protected abstract HDWorldRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry);
}
