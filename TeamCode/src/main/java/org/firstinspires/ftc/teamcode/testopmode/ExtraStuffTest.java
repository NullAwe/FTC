package org.firstinspires.ftc.teamcode.testopmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.PoleDetectionTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;

@Config
@TeleOp(name="testflag.txt", group="super secret")
//@Disabled
public class ExtraStuffTest extends LinearOpMode {

    public static double MOTOR_POWER = 0.0;
    public static double SERVO_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "intakeRotate");
        DcMotor motor = hardwareMap.get(DcMotor.class, "leftRear");
//        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "clawColorSensor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GamePad gp1 = new GamePad(gamepad1);

        telemetry.addData("robot", "initalized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(SERVO_POS);
            motor.setPower(MOTOR_POWER);

            telemetry.addData("motor pos", motor.getCurrentPosition());
//            telemetry.addData("color",
//                    colorSensor.alpha() + " " + colorSensor.red() + " " + colorSensor.green() +
//                            " " + colorSensor.blue());
            telemetry.update();
            gp1.update();
        }
    }
}
