package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.DeliverySlideTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraFrameSource;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraWrapper;
import org.firstinspires.ftc.teamcode.util.objectdetector.ConeDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.CustomSignalDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;
import org.firstinspires.ftc.teamcode.util.objectdetector.SignalDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.VuforiaBasedCameraSource;

public abstract class HDWorldRobotBase extends HDRobotBase {
    private static final String TAG = HDRobotBase.class.getSimpleName();

    // Constant for intake rotate actions
    protected final static double INTAKE_ROTATE_TICKS_PER_RADIAN = 1.0 / Math.toRadians(270.0);
    // Constant for intake lift actions
    private final static double INTAKE_SLIDE_TICKS_PER_REVOLUTION = 145.1; // 1150rpm
    private final static double INTAKE_SLIDE_INCH_PER_REVOLUTION = 4.46;
    private final static double INTAKE_SLIDE_TICKS_PER_INCH =
            INTAKE_SLIDE_TICKS_PER_REVOLUTION / INTAKE_SLIDE_INCH_PER_REVOLUTION;
    // Constant for delivery rotate actions
    protected final static double DELIVERY_ROTATE_TICKS_PER_RADIAN = 4.5 / 5 / 2 / Math.PI;
    // Constant for delivery slide actions
    private final static double DELIVERY_SLIDE_TICKS_PER_REVOLUTION = 145.1; // 1150rpm
    private final static double DELIVERY_SLIDE_INCH_PER_REVOLUTION = 4.42;
    private final static double DELIVERY_SLIDE_TICKS_PER_INCH =
            DELIVERY_SLIDE_TICKS_PER_REVOLUTION / DELIVERY_SLIDE_INCH_PER_REVOLUTION;


    private final Servo intakeClaw;
    private final Servo intakeGuardLeft;
    private final Servo intakeGuardRight;
    private final Servo intakeRotate;
    private final Servo deliveryRotate;
    private final DcMotorEx intakeSlide;
    private final DcMotorEx deliverySlide;
    private final RevColorSensorV3 clawColorSensor;
    private final WebcamName signalWebcam;
    private final WebcamName coneWebcam;
    private boolean isClawOpen = true;
    private ConeDetector coneDetector = null;
    private CameraWrapper cameraWrapper;
    private SignalDetector signalDetector = null;
    private VuforiaLocalizer vuforia = null;

    public HDWorldRobotBase(RobotInitParameters initParameters) {
        super(initParameters);

        HardwareMap hardwareMap = initParameters.hardwareMap;
        deliverySlide = hardwareMap.get(DcMotorEx.class, "deliverySlide");
        intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeGuardLeft = hardwareMap.get(Servo.class, "intakeGuardLeft");
        intakeGuardRight = hardwareMap.get(Servo.class, "intakeGuardRight");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        deliveryRotate = hardwareMap.get(Servo.class, "deliveryRotate");
        clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "clawColorSensor");

        intakeSlide.setMode();

        signalWebcam = null; // hardwareMap.get(WebcamName.class, "signalWebcam");
        coneWebcam = null; // hardwareMap.get(WebcamName.class, "coneWebcam");

        initObjectDetectorResources(hardwareMap);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initObjectDetectorResources(HardwareMap hardwareMap) {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        if (signalWebcam != null) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = signalWebcam;
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            signalDetector = new CustomSignalDetector();
            signalDetector.init(hardwareMap, vuforia, 2);
        }

        if (coneWebcam != null) {
            cameraWrapper = new CameraWrapper(coneWebcam, 52, false);
            if (cameraWrapper.initCamera()) {
                coneDetector = new ConeDetector(cameraWrapper);
            } else {
                RobotLog.ee(TAG, "Failed to initialize cone camera");
            }
        } else {
            RobotLog.ee(TAG, "Failed to initialize claw camera");
        }
    }

    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }

    public ConeDetector getConeDetector() {
        return coneDetector;
    }

    public CameraFrameSource getVuforiaBasedCameraSource() {
        return new VuforiaBasedCameraSource(vuforia, Math.toRadians(44), false);
    }

    public CameraFrameSource getCameraWrapper() {
        return cameraWrapper;
    }

    public void stopCameraWrapper() {
        cameraWrapper.stopCamera();
    }

    public int detectSignal() {
        if (signalDetector == null) return -1;

        return signalDetector.detectSignal();
    }

    public boolean isBlueCone() {
        return ImageProcessor.isBlueWithMinSaturation(ImageProcessor.ColorToHsv(getConeColor()));
    }

    public boolean isRedCone() {
        return ImageProcessor.isRedWithMinSaturation(ImageProcessor.ColorToHsv(getConeColor()));
    }

    public boolean isPole() {
        return ImageProcessor.isYellowWithMinSaturation(ImageProcessor.ColorToHsv(getConeColor()));
    }

    public boolean isCone() {
        return ImageProcessor.isBlueOrRedWithMinSaturation(
                ImageProcessor.ColorToHsv(getConeColor()));
    }

    public NormalizedRGBA getConeColor() {
        return clawColorSensor.getNormalizedColors();
    }

    public double getConeDistanceInch() {
        return clawColorSensor.getDistance(DistanceUnit.INCH);
    }

    // Begin: utils for intake claw action.
    public boolean isClawOpen() {
        return isClawOpen;
    }

    public void toggleClaw() {
        if (isClawOpen()) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public void openClaw() {
        setArmClawPosition(getClawOpenPos());
        isClawOpen = true;
    }

    public void closeClaw() {
        setArmClawPosition(getClawClosePos());
        isClawOpen = false;
    }

    public void setArmClawPosition(double pos) {
        intakeClaw.setPosition(pos);
    }

    public abstract double getClawOpenPos();

    public abstract double getClawClosePos();
    // End: utils for intake claw action.

    // Begin: utils for intake guard action.
    public void guardIntake() {
        setIntakeGuardLeftPosition(getLeftGuardPos());
        setIntakeGuardRightPosition(getRightGuardPos());
    }

    public void unguardIntake() {
        setIntakeGuardLeftPosition(getLeftUnguardPos());
        setIntakeGuardRightPosition(getRightUnguardPos());
    }

    public void setIntakeGuardLeftPosition(double pos) {
        intakeGuardLeft.setPosition(pos);
    }

    public void setIntakeGuardRightPosition(double pos) {
        intakeGuardRight.setPosition(pos);
    }

    public abstract double getLeftGuardPos();
    public abstract double getRightGuardPos();
    public abstract double getLeftUnguardPos();
    public abstract double getRightUnguardPos();
    // End: utils for intake guard action.

    // Begin: utils for intake slide actions
    public Task getIntakeSlideUpTask() {
        return new IntakeSlideTask(this, 12.0);
    }

    public Task getIntakeSlideDownTask() {
        return new IntakeSlideTask(this, 0.0);
    }

    public void setIntakeSlideHeight(double height) {
        intakeSlide.setTargetPosition((int) (height * INTAKE_SLIDE_TICKS_PER_INCH));
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setIntakeSlidePower(double power) {
        intakeSlide.setPower(power);
    }

    public double getIntakeSlideTicksPerInch() {
        return INTAKE_SLIDE_TICKS_PER_INCH;
    }
    // End: utils for intake slide actions

    // Begin: utils for intake rotate actions
    public Task getIntakeRotateDeliveryTask() {
        return new IntakeRotateTask(this, 60.0, AngleType.DEGREE);
    }

    public Task getIntakeRotateResetTask() {
        return new IntakeRotateTask(this, 0.0, AngleType.DEGREE);
    }

    public void setIntakeRotateAngle(double angleRadian) {
        intakeRotate.setPosition(
                angleRadian * getIntakeRotateTicksPerRadian() + getIntakeRotateZeroAnglePos());
    }

    public abstract double getIntakeRotateZeroAnglePos();

    public abstract double getIntakeRotateTicksPerRadian();
    // End: utils for intake rotate actions

    // Begin: utils for delivery slide actions
    public Task getDeliverySlideUpTask() {
        return new DeliverySlideTask(this, 30.0, 0.8, 700);
    }

    public Task getDeliverySlideDownTask() {
        return new DeliverySlideTask(this, 0.0, 0.4, 400);
    }

    public void setDeliverySlideHeight(double height) {
        deliverySlide.setTargetPosition((int) (height * getDeliverySlideTicksPerInch()));
        deliverySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDeliverySlidePower(double power) {
        deliverySlide.setPower(power);
    }

    public double getDeliverySlideTicksPerInch() {
        return -DELIVERY_SLIDE_TICKS_PER_INCH;
    }

    // End: utils for delivery slide actions

    // Begin: utils for delivery rotate actions
    public Task getDeliveryRotateDeliveryTask() {
        return new DeliveryRotateTask(this, 60.0, AngleType.DEGREE);
    }

    public Task getDeliveryRotateResetTask() {
        return new DeliveryRotateTask(this, 0.0, AngleType.DEGREE);
    }

    public void setDeliveryRotateAngle(double angleRadian) {
        deliveryRotate.setPosition(
                angleRadian * getDeliveryRotateTicksPerRadian() + getDeliveryRotateZeroAnglePos());
    }

    public abstract double getDeliveryRotateZeroAnglePos();

    public abstract double getDeliveryRotateTicksPerRadian();
    // End: utils for delivery rotate actions
}