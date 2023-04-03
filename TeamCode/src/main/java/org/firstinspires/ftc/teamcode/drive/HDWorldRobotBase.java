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
import org.firstinspires.ftc.teamcode.task.IntakeRotateTask;
import org.firstinspires.ftc.teamcode.task.IntakeSlideTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraFrameSource;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraWrapper;
import org.firstinspires.ftc.teamcode.util.objectdetector.ConeDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.CustomSignalDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;
import org.firstinspires.ftc.teamcode.util.objectdetector.PoleDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.SignalDetector;
import org.firstinspires.ftc.teamcode.util.objectdetector.VuforiaBasedCameraSource;

public abstract class HDWorldRobotBase extends HDRobotBase {
    // Constant for intake rotate actions
    public static double CONE_RIGHTER_DOWN_ANGLE_DEGREE = 93;
    protected final static double INTAKE_ROTATE_TICKS_PER_RADIAN = 1.0 / Math.toRadians(270.0);
    private static final String TAG = HDRobotBase.class.getSimpleName();
    // Constant for intake lift actions
    private final static double INTAKE_SLIDE_TICKS_PER_REVOLUTION = 145.1; // 1150rpm
    private final static double INTAKE_SLIDE_INCH_PER_REVOLUTION = 4.46;
    protected final static double INTAKE_SLIDE_TICKS_PER_INCH =
            INTAKE_SLIDE_TICKS_PER_REVOLUTION / INTAKE_SLIDE_INCH_PER_REVOLUTION;
    private final Servo intakeClaw;
    private final Servo intakeRotate;
    private final Servo deliveryRotate;
    private final Servo coneRighter;
    private final DcMotorEx intakeSlide;
    private final DcMotorEx deliverySlide;
    private final RevColorSensorV3 clawColorSensor;
    private final WebcamName backWebcam;
    private final WebcamName frontWebcam;
    // Cached Encoder values.
    private int intakeSlidePos = 0, deliverySlidePos = 0; // Encoder Values
    private double intakeSlideVel = 0.0, deliverySlideVel = 0.0; // Velocities
    private boolean isClawOpen = true;
    private boolean isConeRighterUp = true;
    private PoleDetector poleDetector = null;
    private ConeDetector coneDetector = null;
    private CameraFrameSource backCameraSource;
    private CameraWrapper frontCameraSource;
    private SignalDetector signalDetector = null;
    private VuforiaLocalizer vuforia = null;

    public HDWorldRobotBase(RobotInitParameters initParameters) {
        super(initParameters);

        deliverySlide = hardwareMap.get(DcMotorEx.class, "deliverySlide");
        intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        deliveryRotate = hardwareMap.get(Servo.class, "deliveryRotate");
        coneRighter = hardwareMap.get(Servo.class, "coneRighter");
        clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "clawColorSensor");

        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backWebcam = hardwareMap.get(WebcamName.class, "backWebcam");
        frontWebcam = null; // hardwareMap.get(WebcamName.class, "frontWebcam");

        initObjectDetectorResources(hardwareMap);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initObjectDetectorResources(HardwareMap hardwareMap) {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        if (backWebcam != null) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = backWebcam;
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            signalDetector = new CustomSignalDetector();
            signalDetector.init(hardwareMap, vuforia, 2);
            backCameraSource = new VuforiaBasedCameraSource(vuforia, Math.toRadians(52), false);
            poleDetector = new PoleDetector(backCameraSource);
        }
        if (frontWebcam != null) {
            frontCameraSource = new CameraWrapper(frontWebcam, Math.toRadians(52), false);
            if (frontCameraSource.initCamera()) {
                coneDetector = new ConeDetector(frontCameraSource);
            } else {
                RobotLog.ee(TAG, "Failed to initialize front camera");
            }
        } else {
            RobotLog.ee(TAG, "Failed to initialize front camera");
        }
    }

    public void updateEncoderValues() {
        intakeSlidePos = intakeSlide.getCurrentPosition();
        deliverySlidePos = deliverySlide.getCurrentPosition();

        intakeSlideVel = intakeSlide.getVelocity();
        deliverySlideVel = deliverySlide.getVelocity();
        telemetry.addData("Cycle time: ", (int) globalTimer.milliseconds());
        globalTimer.reset();
    }

    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }

    public PoleDetector getPoleDetector() {
        return poleDetector;
    }

    public ConeDetector getConeDetector() {
        return coneDetector;
    }

    public CameraFrameSource getBackCameraSource() {
        return backCameraSource;
    }

    public void stopFrontCameraSource() {
        frontCameraSource.stopCamera();
    }

    public CameraFrameSource getFrontCamerSource() {
        return frontCameraSource;
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

    public void openClawHalf() {
        setArmClawPosition(getClawHalfOpenPos());
        isClawOpen = false;
    }

    public void closeClaw() {
        setArmClawPosition(getClawClosePos());
        isClawOpen = false;
    }

    public void setArmClawPosition(double pos) {
        intakeClaw.setPosition(pos);
    }

    public abstract double getClawOpenPos();
    public abstract double getClawHalfOpenPos();
    public abstract double getClawClosePos();
    // End: utils for intake claw action.

    // Begin: utils for intake slide actions
    public Task getIntakeSlideUpTask() {
        return new IntakeSlideTask(this, 12.0);
    }

    public Task getIntakeSlideDownTask() {
        return new IntakeSlideTask(this, 0.0);
    }

    public void setIntakeSlideHeight(double height) {
        intakeSlide.setTargetPosition((int) (height * getIntakeSlideTicksPerInch()));
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setIntakeSlidePower(double power) {
        intakeSlide.setPower(power);
    }

    public abstract double getIntakeSlideTicksPerInch();

    public double getIntakeSlidePositionInches() {
        return intakeSlidePos / getIntakeSlideTicksPerInch();
    }
    // End: utils for intake slide actions

    // Begin: utils for intake rotate actions
    public Task getIntakeRotateDeliveryTask() {
        return new IntakeRotateTask(this, 60.0, AngleType.DEGREE);
    }

    public Task getIntakeRotateResetTask() {
        return new IntakeRotateTask(this, 0.0, AngleType.DEGREE);
    }

    public double getIntakeRotateAngle() {
        return (intakeRotate.getPosition() - getIntakeRotateZeroAnglePos()) /
                getIntakeRotateTicksPerRadian();
    }

    public void setIntakeRotateAngle(double angleRadian) {
        intakeRotate.setPosition(
                Math.min(Math.max(angleRadian * getIntakeRotateTicksPerRadian() +
                        getIntakeRotateZeroAnglePos(), 0.0), 1.0));
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

    public double getDeliveryRotateAngle() {
        return (deliveryRotate.getPosition() - getDeliveryRotateZeroAnglePos()) /
                getDeliveryRotateTicksPerRadian();
    }

    public void setDeliveryRotateAngle(double angleRadian) {
        deliveryRotate.setPosition(
                angleRadian * getDeliveryRotateTicksPerRadian() + getDeliveryRotateZeroAnglePos());
    }

    public abstract double getDeliverySlideTicksPerInch();

    // End: utils for delivery slide actions

    public double getDeliverySlidePositionInches() {
        return deliverySlidePos / getDeliverySlideTicksPerInch();
    }

    // Begin: utils for delivery rotate actions
    public Task getDeliveryRotateDeliveryTask() {
        return new DeliveryRotateTask(this, 60.0, AngleType.DEGREE);
    }

    public Task getDeliveryRotateResetTask() {
        return new DeliveryRotateTask(this, 0.0, AngleType.DEGREE);
    }

    public abstract double getDeliveryRotateZeroAnglePos();

    public abstract double getDeliveryRotateTicksPerRadian();
    // End: utils for delivery rotate actions

    // Begin: utils for cone righter action
    public boolean isConeRighterUp() {
        return isConeRighterUp;
    }

    public void toggleConeRighter() {
        if (isConeRighterUp()) {
            downConeRighter();
        } else {
            upConeRighter();
        }
    }

    public void initConeRighter() {
        setConeRighterAngle(0);
        isConeRighterUp = true;
    }

    public void upConeRighter() {
        if (isConeRighterUp) return;

        setConeRighterAngle(0);
        isConeRighterUp = true;
    }

    public void downConeRighter() {
        if (!isConeRighterUp) return;

        setConeRighterAngle(Math.toRadians(CONE_RIGHTER_DOWN_ANGLE_DEGREE));
        isConeRighterUp = false;
    }

    public void setConeRighterAngle(double angleRadian) {
        coneRighter.setPosition(
                angleRadian * getConeRighterTicksPerRadian() + getConeRighterUpPos());
    }

    public abstract double getConeRighterUpPos();

    public abstract double getConeRighterTicksPerRadian();
    // End: utils for delivery rotate actions

    // Auto or Tele-op parameters
    public double getIntakeHeightMax() {
        return 19.0;
    }
    public static double INTAKE_HEIGHT_LOW_JUNCTION = 14;
    public double getIntakeHeightLowJunction() {
        return INTAKE_HEIGHT_LOW_JUNCTION;
    }

    public abstract double getAutoIntakeDeliveryHeightInch();
    public abstract double getTeleopIntakeDeliveryHeightInch();
    public abstract double getIntakeDeliveryRotateDegree();
    public abstract int getIntakeSlideDropUpDelay();
    public abstract double getDeliveryHeightHigh();
    public abstract double getDeliveryHeightMedium();
    public double getDeliveryHeightLow() {
        return 11.5;
    }
    public abstract double getAutoDeliveryRotateAngleDegree();
}