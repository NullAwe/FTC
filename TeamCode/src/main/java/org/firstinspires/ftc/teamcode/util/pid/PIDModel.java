package org.firstinspires.ftc.teamcode.util.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PIDModel {
    private final double minPower;
    private final double maxPower;
    private final double kP;
    private final double kI;
    private final double kD;

    protected double lastTarget = Integer.MAX_VALUE;

    public PIDModel(double minPower, double maxPower, double pK, double iK, double dK) {
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.kP = pK;
        this.kI = iK;
        this.kD = dK;
    }

    private ElapsedTime timer = null;
    private double lastError = 1e9;
    private double speed = 0.0;
    private double integral = 0.0;
    private boolean integralStarted = false;

    public double getPower() {
        double error = getError();
        if (timer == null) {
            lastError = error;
            timer = new ElapsedTime();
        }
        double deltaT = timer.milliseconds();
        timer.reset();
        speed = (error - lastError) / deltaT;

//        if (integralStarted) {
//            integral += error * deltaT;
//        }
        double power = kP * error + kI * integral + speed * kD;

        double absolutePower = Math.abs(power);
        if (absolutePower > maxPower) {
            power = power > 0 ? maxPower : -maxPower;
        } else {
//            if (!integralStarted){
//                integralStarted = true;
//            }
            if (absolutePower < minPower && Math.abs(speed) < getStopSpeed() &&
                    Math.abs(error) > getStopError()) {
                power = error > 0 ? minPower : -minPower;
            }
        }
        lastError = error;
        return power;
    }

    public boolean isDone() {
        return Math.abs(lastError) < getStopError() && Math.abs(speed) < getStopSpeed();
    }

    public abstract void cancel();
    // Need to be public for PID tuning tools
    public abstract double getError();
    protected abstract double getStopError();
    protected abstract double getStopSpeed();
}
