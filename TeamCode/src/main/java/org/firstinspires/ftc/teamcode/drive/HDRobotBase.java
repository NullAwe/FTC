package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public abstract class HDRobotBase extends MecanumDrive {
    protected static final String VUFORIA_KEY =
            "AXmxi+b/////AAABmazWyYO7aUlrjrYb3f16ZIMIutEjCylZIDW82luhoGYwENOMgUOxnJmdyIe3jatwyBUoTkRfL8nbIjWeN36jIM8CYSVWLxFlsGiLUe8oTmt4Pi6qn82nAJEXIZsGbAH27NP0Rc2XID0DO1qx6e8c0ahBUzgrZrhF6On0rR7dSZLStNnEENzzWX4qPsawW7IWROP8s7SAvdUgXrXe5cX+/e60NPg6mEw61SfPS/3UP3y3mz7DBfhJRC/H26rL2iZg7aosUIJkSf1gjyp9hRbvznfQX/zDrB3tXexJ2KBUTrHEzDnamVk7SMXq7ltiZyoDiaB6qL6BGgpvuQ9X6Oh3Ahmzch93IZSANif6KxvclNXg";
    private static final String TAG = HDRobotBase.class.getSimpleName();
    public static int ACTIVE_ROBOT = 1;
    private final RoadRunnerParameters roadRunnerParameters;
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryVelocityConstraint velConstraint;
    private final TrajectoryAccelerationConstraint accelConstraint;
    private final DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private final List<DcMotorEx> motors;
    private final VoltageSensor batteryVoltageSensor;
    protected IMU imu;
    private final List<Integer> lastEncPositions = new ArrayList<>();
    private final List<Integer> lastEncVels = new ArrayList<>();
    private double total = 0.0;
    private int num = 0;

    public HDRobotBase(RobotInitParameters initParameters) {
        super(initParameters.roadRunnerParams.feedforwardParams.kV,
                initParameters.roadRunnerParams.feedforwardParams.kA,
                initParameters.roadRunnerParams.feedforwardParams.kStatic,
                initParameters.roadRunnerParams.trackWidth,
                initParameters.roadRunnerParams.trackWidth,
                initParameters.roadRunnerParams.lateralMultiplier);

        this.roadRunnerParameters = initParameters.roadRunnerParams;
        HardwareMap hardwareMap = initParameters.hardwareMap;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                roadRunnerParameters.logoFacingDirection, roadRunnerParameters.usbFacingDirection));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        velConstraint =
                getVelocityConstraint(roadRunnerParameters.maxVel,
                        roadRunnerParameters.maxAngVel,
                        roadRunnerParameters.trackWidth);
        accelConstraint = getAccelerationConstraint(roadRunnerParameters.maxAccel);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (roadRunnerParameters.runUsingEncoder) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (roadRunnerParameters.runUsingEncoder && roadRunnerParameters.motorVeloPID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    roadRunnerParameters.motorVeloPID);
        }

        if (roadRunnerParameters.usingTwoWheelTracking) {
            setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));
        }

        TrajectoryFollower follower =
                new HolonomicPIDVAFollower(roadRunnerParameters.translationalPID,
                        roadRunnerParameters.translationalPID, roadRunnerParameters.headingPID,
                        new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, roadRunnerParameters.headingPID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels,
                roadRunnerParameters.runUsingEncoder);
    }

    public double rpmToVelocity(double rpm) {
        return roadRunnerParameters.rpmToVelocity(rpm);
    }

    public RoadRunnerParameters getRoadRunnerParameters() {
        return roadRunnerParameters;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel,
            double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public IMU getImu() {
        return imu;
    }

    public void setVelocities(double... vels) {
        if (vels.length != 4) return;
        for (int i = 0; i < 4; ++i) {
            motors.get(i).setVelocity(vels[i]);
        }
    }

    // Setting the powers of motors:
    public void setPowers(double... powers) {
        if (powers.length != 4) return;
        for (int i = 0; i < 4; ++i) {
            motors.get(i).setPower(powers[i]);
        }
    }

    public double[] getPowers(double power, double angle) {
        angle += Math.PI * 0.25;
        double xp = Math.cos(angle);
        double yp = Math.sin(angle);
        double factor = 0.98 / Math.max(Math.abs(xp), Math.abs(yp));
        xp *= factor * power;
        yp *= factor * power;
        return new double[]{xp, yp, yp, xp};
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                velConstraint, accelConstraint,
                roadRunnerParameters.maxAngVel, roadRunnerParameters.maxAngAccel
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        ElapsedTime timer = new ElapsedTime();
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
        total += timer.milliseconds();
        num++;
        RobotLog.ee("CycleTime", "Roadrunner: %d, average: %d",
                (int) (timer.milliseconds()), (int) (total / num));
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = roadRunnerParameters.vxWeight * Math.abs(drivePower.getX())
                    + roadRunnerParameters.vyWeight * Math.abs(drivePower.getY())
                    + roadRunnerParameters.omegaWeight * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    roadRunnerParameters.vxWeight * drivePower.getX(),
                    roadRunnerParameters.vyWeight * drivePower.getY(),
                    roadRunnerParameters.omegaWeight * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(roadRunnerParameters.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(roadRunnerParameters.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public abstract Double getExternalHeadingVelocity();
}
