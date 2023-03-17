package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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

/** TeleOp Base Class with complete different functionalities. */
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

    // Testing
    public static int INTAKE_ROTATE_DELAY_MILLIS = 400;
    public static int INTAKE_DELIVERY_PAUSE_MILLIS = 40;
    public static int DELIVERY_DELAY_MILLIS = 300;

    protected final ElapsedTime cycleTime = new ElapsedTime();
    private final FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    protected HDWorldRobotBase robot;
    private final ElapsedTime releaseTime = new ElapsedTime();
    private Pose2d prevPower = new Pose2d(0.0, 0.0, 0.0);
    private Task presetTask = null;

    private double intakeHeight = 0.0;
    private double deliveryHeight = 0.0;
    private double deliveryAngle = 0.0;

    public static double DELIVERY_ANGLE_ROTATE_RADIAN = 1.4;

    //
    GamePad gp1, gp2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        robot = createRobot(hardwareMap);

        robot.setIntakeRotateAngle(0.0);
        robot.openClaw();
        robot.setDeliveryRotateAngle(deliveryAngle);

        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        waitForStart();

        cycleTime.reset();
        while (opModeIsActive()) {
            driveRobot(gp1);

            if (presetTask != null) {
                if (gp1.onceX()) {
                    presetTask.cancel();
                    presetTask = null;
                } else if (presetTask.perform()) {
                    presetTask = null;
                }
            } else {
                if (gp2.onceX()) {
                    // Open or close claw/gripper
                    robot.toggleClaw();
                    releaseTime.reset();
                } else {
                    doIntake();
                    doDelivery();
                    if (robot.isCone() && robot.getConeDistanceInch() < MAX_CONE_GRABBING_DIST &&
                            robot.isClawOpen() && releaseTime.milliseconds() > MIN_GAP_MILLIS) {
                        presetTask = getSecureConeTask();
                    }
                }
            }

            // Test:
            telemetry.update();

            gp1.update();
            gp2.update();
            if (isStopRequested()) {
                try {
                    robot.stopCameraWrapper();
                    RobotLog.ee("TeleOpBase", "Shutting down the camera stream");
                } catch (Exception e) {
                    RobotLog.ee("TeleOpBase", e, "failed to shut down camera");
                }
            }
        }
    }

    private void doIntake() {
        if (gp2.onceB() && !robot.isClawOpen()) {
            presetTask = getPickUpConeTask();
        } else if (gp2.onceA()) {
            presetTask = getResetIntakeTask();
        }
        if (presetTask == null) {
            if (Math.abs(gp2.rightStickY()) > 0.1 && Math.abs(gp2.rightStickY()) > Math.abs(gp2.rightStickX())) {
                intakeHeight -= gp2.rightStickY() / 0.2;
                intakeHeight = Math.max(Math.min(intakeHeight, 20), -1);
                presetTask = new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
            } else if (gp2.onceDpadUp()) {
                intakeHeight += 1.25;
                intakeHeight = Math.max(Math.min(intakeHeight, 20), -1);
                presetTask = new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
            } else if (gp2.onceDpadDown()) {
                intakeHeight -= 1.25;
                intakeHeight = Math.max(Math.min(intakeHeight, 20), -1);
                presetTask = new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER);
            } else if (Math.abs(gp2.rightStickX()) > 0.1) {
                robot.setIntakeRotateAngle(robot.getIntakeRotateAngle() + gp2.rightStickX() / 50);
            }
        }
    }

    private void doDelivery() {
        if (gp2.rightBumper() && gp2.leftBumper()) {
            deliveryHeight = 0;
            presetTask = new DeliverySlideTask(robot, deliveryHeight, DELIVERY_POWER);
        } else if (gp2.onceRightBumper()) {
            deliveryHeight = getDeliveryHeightHigh();
            presetTask = new ParallelTask(
                    new SeriesTask(
                            new SleepTask(DELIVERY_DELAY_MILLIS),
                            new DeliverySlideTask(robot, deliveryHeight, DELIVERY_POWER)),
                    getResetIntakeTask());
        } else if (gp2.onceLeftBumper()) {
            deliveryHeight = getDeliveryHeightMedium();
            presetTask = new ParallelTask(
                    new SeriesTask(
                            new SleepTask(DELIVERY_DELAY_MILLIS),
                            new DeliverySlideTask(robot, deliveryHeight, DELIVERY_POWER)),
                    getResetIntakeTask());
        }
        if (presetTask == null) {
            if (Math.abs(gp2.leftStickY()) > 0.1) {
                deliveryHeight -= gp2.leftStickY() / 0.2;
//                deliveryHeight = Math.max(Math.min(deliveryHeight, 23), -1);
                deliveryHeight = Math.max(Math.min(deliveryHeight, 23), -1);
                presetTask = new DeliverySlideTask(robot, deliveryHeight, DELIVERY_POWER);
            }
        }
        if (gp2.leftTrigger() > 0.1 && gp2.rightTrigger() > 0.1) {
            deliveryAngle = 0;
        } else if (gp2.onceLeftTrigger()) {
            deliveryAngle = DELIVERY_ANGLE_ROTATE_RADIAN;
        } else if (gp2.onceRightTrigger()) {
            deliveryAngle = -DELIVERY_ANGLE_ROTATE_RADIAN;
        }
        robot.setDeliveryRotateAngle(deliveryAngle);
    }

    private Task getPickUpConeTask() {
        intakeHeight = this.getIntakeDeliveryHeightInch();
        return new ParallelTask(
                        new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER),
                        new SeriesTask(
                                new SleepTask(INTAKE_ROTATE_DELAY_MILLIS),
                                new IntakeRotateTask(robot, getIntakeDeliveryRotateDegree(),
                                        AngleType.DEGREE)));
    }
    private Task getSecureConeTask() {
        intakeHeight = 2;
        return new SeriesTask(
                new IntakeClawTask(robot, /*open=*/false),
                new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER));
    }

    private Task getResetIntakeTask() {
        intakeHeight = 0;
        return new SeriesTask(
                new IntakeClawTask(robot, true),
                new ParallelTask(
                        new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                        new IntakeClawTask(robot, false)),
                new IntakeSlideTask(robot, intakeHeight, INTAKE_SLIDER_POWER),
                new IntakeClawTask(robot, true));
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

    protected abstract HDWorldRobotBase createRobot(HardwareMap hardwareMap);

    protected abstract double getIntakeDeliveryHeightInch();
    protected abstract double getIntakeDeliveryRotateDegree();
    protected abstract double getDeliveryHeightHigh();
    protected abstract double getDeliveryHeightMedium();
}
