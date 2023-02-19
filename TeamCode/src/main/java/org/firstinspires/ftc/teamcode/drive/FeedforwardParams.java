package org.firstinspires.ftc.teamcode.drive;

public class FeedforwardParams {
    public double kV; //1.0 / rpmToVelocity(MAX_RPM);
    public double kA;
    public double kStatic;

    public FeedforwardParams(double v, double a, double kStatic) {
        this.kV = v;
        this.kA = a;
        this.kStatic = kStatic;
    }
}
