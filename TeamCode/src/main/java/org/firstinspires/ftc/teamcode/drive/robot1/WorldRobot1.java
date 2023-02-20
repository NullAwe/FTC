package org.firstinspires.ftc.teamcode.drive.robot1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.FeedforwardParams;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.RoadRunnerParameters;
import org.firstinspires.ftc.teamcode.drive.RobotInitParameters;

@Config
public class WorldRobot1 extends HDWorldRobotBase {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    /*
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 10, 13.9);
    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1.03; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 11.6; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static FeedforwardParams FEEDFORWARD_PRAMS = new FeedforwardParams(
            1 / (MAX_RPM * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0), 0.0, 0.0);
//    public static FeedforwardParams FEEDFORWARD_PRAMS = new FeedforwardParams(0.0135, 0.004, 0.0);

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 72;
    public static double MAX_ACCEL = 72;
    public static double MAX_ANG_VEL = 5.9; //Math.toRadians(262.03258124999996);
    public static double MAX_ANG_ACCEL = 5.9;

    /*
     Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     If the hub containing the IMU you are using is mounted so that the "REV" logo does
     not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)

                 | +Z axis
                 |
                 |
                 |
          _______|_____________     +Y axis
         /       |_____________/|__________
        /   REV / EXPANSION   //
       /       / HUB         //
      /_______/_____________//
     |_______/_____________|/
            /
           / +X axis

     This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec
     .com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
     and the placement of the dot/orientation from https://docs.revrobotics
     .com/rev-control-system/control-system-overview/dimensions#imu-location

     For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
     BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 0.1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0.0);
    public static double LATERAL_MULTIPLIER = 1.184;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public WorldRobot1(HardwareMap hardwareMap) {
        super(new RobotInitParameters(hardwareMap, createRoadRunnerParameters()));

        // Dummy code to make sure this is indeed robot 1
        Servo nonExistServo = hardwareMap.get(Servo.class, "for_robot_1_only");
    }

    private static RoadRunnerParameters createRoadRunnerParameters() {
        RoadRunnerParameters params = new RoadRunnerParameters();
        params.runUsingEncoder = true;
        params.usingTwoWheelTracking = false;
        params.motorVeloPID = MOTOR_VELO_PID;
        params.ticksPerRev = TICKS_PER_REV;
        params.maxRPM = MAX_RPM;
        params.wheelRadius = WHEEL_RADIUS;
        params.gearRatio = GEAR_RATIO;
        params.trackWidth = TRACK_WIDTH;
        params.feedforwardParams = FEEDFORWARD_PRAMS;
        params.maxVel = MAX_VEL;
        params.maxAccel = MAX_ACCEL;
        params.maxAngVel = MAX_ANG_VEL;
        params.maxAngAccel = MAX_ANG_ACCEL;
        params.logoFacingDirection = LOGO_FACING_DIR;
        params.usbFacingDirection = USB_FACING_DIR;
        params.translationalPID = TRANSLATIONAL_PID;
        params.headingPID = HEADING_PID;
        params.lateralMultiplier = LATERAL_MULTIPLIER;
        params.vxWeight = VX_WEIGHT;
        params.vyWeight = VY_WEIGHT;
        params.omegaWeight = OMEGA_WEIGHT;
        return params;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;
    }

    @Override
    public double getClawOpenPos() {
        return 0.68;
    }

    @Override
    public double getClawClosePos() {
        return 0.5;
    }

    @Override
    public double getLeftGuardPos() {
        return 0.68;
    }

    @Override
    public double getRightGuardPos() {
        return 0.68;
    }

    @Override
    public double getLeftUnguardPos() {
        return 0.3;
    }

    @Override
    public double getRightUnguardPos() {
        return 0.3;
    }

    @Override
    public double getIntakeRotateZeroAnglePos() {
        return 0.2;
    }

    @Override
    public double getIntakeRotateTicksPerRadian() {
        return INTAKE_ROTATE_TICKS_PER_RADIAN;
    }

    @Override
    public double getDeliveryRotateZeroAnglePos() {
        return 0.2;
    }

    @Override
    public double getDeliveryRotateTicksPerRadian() {
        return DELIVERY_ROTATE_TICKS_PER_RADIAN;
    }
}