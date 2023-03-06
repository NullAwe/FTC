/* Copyright (c) 2019 Phil Malone. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.util.drive_algorithms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.HDRobotBase;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

import java.util.List;
public class DriveWithPID {

    public static double TICKS_PER_REV = 384.5;
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1.03; // output (wheel) speed / input (motor) speed

    IMU imu;
    double lastAngles = 0;
    double                  globalAngle, power = .50, correction, rotation;
    boolean                 aButton, bButton;
    PIDController           pidRotate, pidDrive;

    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended forward-backward motion when sideways motion is requested.
    protected static final double V_forward_V_side_bias = 0.0;
    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended turn motion when sideways motion is requested.
    protected static final double V_turn_V_side_bias = 0.0;

    protected static final double tics_per_inch_forward = 84.0;
    protected static final double tics_per_inch_backward = 84.0;

    private final DcMotor leftRear, leftFront, rightRear, rightFront;

    public DriveWithPID(HDRobotBase robot) {
        leftFront = robot.getLeftFront();
        leftRear = robot.getLeftRear();
        rightFront = robot.getRightFront();
        rightRear = robot.getRightRear();
        imu = robot.getImu();
        pidDrive = new PIDController(10.0, 0.0, 0.0);
        pidRotate = new PIDController(1.0, 0.0, 0.0);
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double angles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = angles - lastAngles;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public boolean rotate(int degrees)
    {
        double power = 0.8;
        // restart imu angle tracking.
        resetAngle();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        pidRotate.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(power/degrees);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint. Started with 100 but robot did not
        // slow and overshot the turn. Increasing I slowed the end of the turn and completed
        // the turn in a timely manner
        double i = p / 180.0;
        if (degrees < 0)
            i = p / 130.0;

        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
                setPower(-power, -power, power, power);
            }
        }
        power = pidRotate.performPID(getAngle());
        setPower(power, power, -power, -power);
        if (pidRotate.onTarget()) {
            rotation = getAngle();
            resetAngle();
            return true;
        }

        return false;
    }

    /**
     * Get the forward tics, which is the sum of the encoders for all
     * motors. This is a start on redundancy (using all 4 encoders),
     * however, if an encoder fails this will still not detect that.
     */
    private double forward_tics() {
        return rightFront.getCurrentPosition() + leftFront.getCurrentPosition() +
                rightRear.getCurrentPosition() + leftRear.getCurrentPosition();
    }

    /**
     * Get the sideways tics which is the sum if the encoders corrected
     * for direction of rotation when moving sideways. This is a start on
     * redundancy (using all 4 encoders), however, if an encoder fails
     * this will still not detect that.
     */
    private double sideways_tics() {
        return (rightRear.getCurrentPosition() + leftFront.getCurrentPosition()) -
                (rightFront.getCurrentPosition() + leftRear.getCurrentPosition());
    }

    public void setSpeeds(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        double scale = 1.0;
        double max = Math.abs(forward) +
                Math.abs(sideways * (Math.abs(V_forward_V_side_bias) + Math.abs(V_turn_V_side_bias) + 1.0)) +
                Math.abs(rotate);
        if (max > 1.0) {
            scale = 1.0 / max;
        }
        // Compute the power to each of the motors
        double power_rf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias - 1.0) -
                        rotate);
        double power_rr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias + 1.0) -
                        rotate);
        double power_lf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias + 1.0) +
                        rotate);
        double power_lr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias - 1.0) +
                        rotate);
        // set the powers to each of the motors
        setPower(power_rf, power_rr, power_lf, power_lr);

    }

    public void setTankSpeeds(double left, double right, double sideways) {
        // find a scale that makes sure none of the power components is greater than 1 or less than -1
        double scale = 1.0;
        double max_left = Math.abs(left) + Math.abs(sideways);
        double max_right = Math.abs(right) + Math.abs(sideways);
        if (max_left > 1.0) {
            if (max_right > max_left) {
                scale = 1.0 / max_right;
            } else {
                scale = 1.0 / max_left;
            }
        } else if (max_right > 1.0) {
            scale = 1.0 / max_right;
        }
        // set the powers to each of the motors
        setPower(scale * (right - sideways),
                scale * (right + sideways),
                scale * (left + sideways),
                scale * (left - sideways));
    }

    public void setPower(double power_rf, double power_rr, double power_lf, double power_lr)
    {
        leftFront.setPower(power_lf);
        leftRear.setPower(power_lr);
        rightFront.setPower(power_rf);
        rightRear.setPower(power_rr);
    }

    public boolean move(double dist) {
        reset_drive_encoders();
        double forward_direction_mult = (dist > 0.0) ? 1.0 : -1.0;
        double tics_per_inch = (dist > 0.0) ? tics_per_inch_forward : tics_per_inch_backward;
        double forward_target_tics = tics_per_inch * dist *
                forward_direction_mult;
        double current_forward_tics = forward_direction_mult * forward_tics();
        double progress = current_forward_tics / forward_target_tics;
        if (progress < 0.01)
            power = 0.1;
        else
            power = 0.9;
        // Use PID with imu input to drive in a straight line.
        correction = pidDrive.performPID(getAngle());

        if (dist > 0.0) {
            //forward
            // set power levels. working 12in/0.4s with cycles < 2 power = 0.1 else 0.5
                /*leftFront.setPower(power + correction);
                leftRear.setPower(power - correction);
                rightFront.setPower(power - correction);
                rightRear.setPower(power + correction);*/
            setTankSpeeds(power, power, correction);
            //setSpeeds(power, correction, 0);
        } else {
            //Backward working
                /*leftFront.setPower(-power + correction);
                leftRear.setPower(-power - correction);
                rightFront.setPower(-power - correction);
                rightRear.setPower(-power + correction);*/
            setTankSpeeds(-power, -power, correction);
        }

        current_forward_tics = forward_direction_mult * forward_tics();

        setPower(0, 0, 0, 0);

        RobotLog.ee("1 imu heading", "imu heading=%5.1f", lastAngles);
        RobotLog.ee("2 global heading", "global heading=%5.1f", globalAngle);
        RobotLog.ee("3 correction", "correction=%5.1f", correction);
        RobotLog.ee("4 turn rotation", "turn rotation=%5.1f", rotation);
        RobotLog.ee("5 current_forward_tics", "current_forward_tics=%5.1f", current_forward_tics);
        RobotLog.ee("6 forward_target_tics", "forward_target_tics=%5.1f", forward_target_tics);

        return current_forward_tics > forward_target_tics;

    }

    public void Strafe(double inches)
    {
        int cycles = 0;
        reset_drive_encoders();
        double forward_direction_mult = (inches > 0.0) ? 1.0 : -1.0;
        double tics_per_inch = (inches > 0.0) ? tics_per_inch_forward : tics_per_inch_backward;
        double forward_target_tics = tics_per_inch * inches *
                forward_direction_mult;
        double current_forward_tics = forward_direction_mult * sideways_tics();

        while (current_forward_tics < forward_target_tics)
        {
            cycles++;
            double progress = current_forward_tics / forward_target_tics;
            if (cycles < 2)
                power = 0.1;
            else
                power = 0.9;
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            if (inches > 0.0) {
                //left
                leftFront.setPower(-power + correction);
                leftRear.setPower(power - correction);
                rightFront.setPower(-power - correction);
                rightRear.setPower(power + correction);
            }
            else {
                //right
                leftFront.setPower(power - correction);
                leftRear.setPower(-power - correction);
                rightFront.setPower(-power - correction);
                rightRear.setPower(power - correction);
            }

            current_forward_tics = forward_direction_mult * forward_tics();
        }

        setPower(0,0,0,0);
        RobotLog.ee("1 imu heading","imu heading=%5.1f", lastAngles);
        RobotLog.ee("2 global heading","global heading=%5.1f", globalAngle);
        RobotLog.ee("3 correction", "correction=%5.1f",correction);
        RobotLog.ee("4 turn rotation", "turn rotation=%5.1f", rotation);
        RobotLog.ee("5 current_forward_tics", "current_forward_tics=%5.1f", current_forward_tics);
        RobotLog.ee("6 forward_target_tics", "forward_target_tics=%5.1f", forward_target_tics);
        RobotLog.ee("7 cycles", "%d Cycles", cycles);
    }

    /**
     * Reset the drive encoders to 0 and set all motors to RUN_USING_ENCODER,
     * which means that when you set a motor power it is really the motor speed
     * and the motor control libraries use the encoders and PID loops to keep
     * the motors coordinated.
     */
    protected void reset_drive_encoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

}

