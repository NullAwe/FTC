package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RoadRunnerParameters {
    // Drive constants
    public boolean runUsingEncoder;
    public boolean usingTwoWheelTracking;
    public PIDFCoefficients motorVeloPID;
    public double ticksPerRev;
    public double maxRPM;
    public double wheelRadius;
    public double gearRatio;
    public double trackWidth;
    public FeedforwardParams feedforwardParams;
    public double maxVel;
    public double maxAccel;
    public double maxAngVel;
    public double maxAngAccel;
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;

    // PID coefficient
    public PIDCoefficients translationalPID;
    public PIDCoefficients headingPID;
    public double lateralMultiplier;
    public double vxWeight;
    public double vyWeight;
    public double omegaWeight;

    public double encoderTicksToInches(double ticks) {
        return wheelRadius * 2 * Math.PI * gearRatio * ticks / ticksPerRev;
    }

    public double rpmToVelocity(double rpm) {
        return rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
