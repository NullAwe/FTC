package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.drive.RobotFactory.createRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraDebugger;
import org.firstinspires.ftc.teamcode.util.objectdetector.ConeDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.DetectorBase;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;
import org.firstinspires.ftc.teamcode.util.objectdetector.PoleDetector;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
@TeleOp(name = "Device Tester", group = "Test")
@Config
public class DeviceTester extends LinearOpMode {

    public static double OTHER_MOTOR_POWER = 0.4;
    public static int ACTIVE_CAMERA = -1; // 0: arm camera, 1: claw camera, others: none

    public static class ServoPos {
        public double intakeClaw = 0.55;
        public double intakeRotate = 0.41;
        public double deliveryRotate = 0.37;
        public double coneRighter = 0.37;
    }

    public static class MotorPos {
        public double intakeSlide = 0.0;
        public double deliverySlide = 0.0;
    }

    public static class DriveMotorPower {
        public double leftFront = 0.0;
        public double rightFront = 0.0;
        public double leftRear = 0.0;
        public double rightRear = 0.0;
    }

    public static ServoPos SERVO_POS = new ServoPos();
    public static MotorPos MOTOR_POS = new MotorPos();
    public static DriveMotorPower DRIVE_MOTOR_POWER = new DriveMotorPower();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private HDWorldRobotBase robot;
    private GamePad gp1;

    @Override
    public void runOpMode() {
        robot = (HDWorldRobotBase) createRobot(hardwareMap, telemetry);
        CameraDebugger armCameraDebugger = robot.getBackCameraSource() == null ?
                null : new CameraDebugger(robot.getBackCameraSource());
        PoleDetector wristCameraDebugger = robot.getPoleDetector();
        ConeDetector coneCameraDebugger = robot.getConeDetector();
        SERVO_POS.intakeClaw = robot.getClawOpenPos();
        SERVO_POS.intakeRotate = robot.getIntakeRotateZeroAnglePos();
        SERVO_POS.deliveryRotate = robot.getDeliveryRotateZeroAnglePos();

        gp1 = new GamePad(gamepad1);
        Field[] servoNames = SERVO_POS.getClass().getFields();
        Map<String, Servo> servos = new HashMap<>();
        for (Field field : servoNames) {
            servos.put(field.getName(), hardwareMap.get(Servo.class, field.getName()));
        }

        Field[] motorNames = MOTOR_POS.getClass().getFields();
        Map<String, DcMotorEx> motors = new HashMap<>();
        for (Field field : motorNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, field.getName());
            motors.put(field.getName(), motor);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        Field[] driveMotorPowerNames = DRIVE_MOTOR_POWER.getClass().getFields();
        Map<String, DcMotorEx> driveMotorPowers = new HashMap<>();
        for (Field field : driveMotorPowerNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, field.getName());
            driveMotorPowers.put(field.getName(), motor);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry,
                dashboard.getTelemetry());

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            robot.updateEncoderValues();
            try {
                for (Field field : servoNames) {
                    Servo servo = servos.get(field.getName());
                    if (servo != null) {
                        servo.setPosition(field.getDouble(SERVO_POS));
                    }
                }
                for (Field field : motorNames) {
                    DcMotor motor = motors.get(field.getName());
                    if (motor != null) {
                        motor.setTargetPosition(toMotorTicks(field.getName(),
                                field.getDouble(MOTOR_POS)));
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(OTHER_MOTOR_POWER);
                    }
                }
                for (Field field : driveMotorPowerNames) {
                    DcMotor motor = driveMotorPowers.get(field.getName());
                    if (motor != null) {
                        motor.setPower(field.getDouble(DRIVE_MOTOR_POWER));
                        int ticks = motor.getCurrentPosition();
                        multipleTelemetry.addData("Drive " + field.getName(), "%s ticks: %d",
                                field.getName(), ticks);
                    }
                }

                DetectorBase.Segment seg = new DetectorBase.Segment(0, 0);
                PoleDetector.AngleAndDist angleAndDist = new PoleDetector.AngleAndDist();
                double coneAngle = 0.0;
                if (ACTIVE_CAMERA == 0 && armCameraDebugger != null) {
                    seg = armCameraDebugger.showFrame();
                } else if (ACTIVE_CAMERA == 1 && wristCameraDebugger != null) {
                    angleAndDist = wristCameraDebugger.getPoleAngleAndDist();
                } else if (ACTIVE_CAMERA == 2 && coneCameraDebugger != null) {
                    coneAngle = coneCameraDebugger.getConeAngle();
                }

//                robot.setWeightedDrivePower(
//                        new Pose2d(-gp1.leftStickY(), -gp1.leftStickX(), -gp1.rightStickX()));

                HSV hsv = ImageProcessor.ColorToHsv(robot.getConeColor());
                multipleTelemetry.addData("angles", "intake angle: %f, delivery angle: %f",
                        Math.toDegrees(robot.getIntakeRotateAngle()),
                        Math.toDegrees(robot.getDeliveryRotateAngle()));

                multipleTelemetry.addData("heights", "intake height: %f, delivery height: %f",
                        Math.toDegrees(robot.getIntakeSlidePositionInches()),
                        Math.toDegrees(robot.getDeliverySlidePositionInches()));

                multipleTelemetry.addData("color sensor", "H: %d, S: %d, V: %d, dist: %f",
                        (int) hsv.h, (int) hsv.s, (int) hsv.v, robot.getConeDistanceInch());
                multipleTelemetry.addData("cone color",
                        "blue: %s, red: %s, yellow: %s",
                        Boolean.toString(robot.isBlueCone()),
                        Boolean.toString(robot.isRedCone()),
                        Boolean.toString(robot.isPole()));

                HSV hsv2 = ImageProcessor.ColorToHsv(robot.getDeliveryColor());
                multipleTelemetry.addData("delivery color sensor", "H: %d, S: %d, V: %d, dist: %f",
                        (int) hsv2.h, (int) hsv2.s, (int) hsv2.v, robot.getDeliveryDistanceInch());
                multipleTelemetry.addData("delivery cone color",
                        "blue: %s, red: %s",
                        Boolean.toString(robot.isBlueCone(hsv2)),
                        Boolean.toString(robot.isRedCone(hsv2)));

                multipleTelemetry.addData("delivery has cone",
                        Boolean.toString(robot.deliveryHasCone()));

                multipleTelemetry.addData("IMU ori",
                        "Heading: %f, hV: %f, xAV: %f, yAV: %f, zAV: %f",
                        robot.getExternalHeading(),
                        robot.getExternalHeadingVelocity(),
                        robot.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate,
                        robot.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate,
                        robot.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
                multipleTelemetry.addData("obj pos", "pole size: %d and pos: %d",
                        seg.length(), seg.midPoint());
                multipleTelemetry.addData("detection results", "pole angle: %f and dist: %f",
                        Math.toDegrees(angleAndDist.angle), angleAndDist.dist);
                multipleTelemetry.addData("cone results", "cone angle: %f",
                        Math.toDegrees(coneAngle));
                 multipleTelemetry.addData("Time interval", "time: %d",
                        (int)timer.milliseconds());
                timer.reset();

                multipleTelemetry.update();
                gp1.update();
            } catch (IllegalArgumentException | IllegalAccessException e) {
                RobotLog.ee("hd18225", e.getMessage());// ignore
            }
        }

        try {
            robot.stopFrontCameraSource();
        } catch (Exception e) {
            RobotLog.ee("Servo Tester", e, "failed to shut down camera");
        }
    }

    private int toMotorTicks(String name, double height) {
        int ticks = 0;
        if (name.equals("intakeSlide")) {
            ticks = (int) (height *  robot.getIntakeSlideTicksPerInch());
        } else if (name.equals("deliverySlide")) {
            ticks = (int) (height * robot.getDeliverySlideTicksPerInch());
        }
        return ticks;
    }
}