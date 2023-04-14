package org.firstinspires.ftc.teamcode.testopmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.drive.robot1.WorldRobot1;
import org.firstinspires.ftc.teamcode.task.DeliveryRotateTask;
import org.firstinspires.ftc.teamcode.task.PoleDetectionTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name="flag.txt", group="super secret")
public class TopSecretOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HDWorldRobotBase robot = new WorldRobot1(hardwareMap, telemetry);
        AutoStates autoStates = new AutoStates.Builder().build();
        GamePad gp1 = new GamePad(gamepad1);
        Task initTask = new DeliveryRotateTask(robot,
                robot.getAutoDeliveryRotateAngleDegree(),
                AngleType.DEGREE);

        while (!initTask.perform());

        telemetry.addData("robot", "initalized");
        telemetry.update();

        waitForStart();

        Task task = null;

        while (opModeIsActive()) {
            if (gp1.onceA()) {
                if (task != null) task.cancel();
                task = new PoleDetectionTask(robot, autoStates);
                Log.i("allendebug", "created");
            }
            if (task != null && task.perform()) {
                Log.i("allendebug", "finished");
                task = null;
            }
            telemetry.addData("task", task == null ? "null" : task.toString());
            telemetry.addData("pole angle",
                    autoStates.getPoleAngleAndDist() == null ? "null" :
                            Double.toString(autoStates.getPoleAngleAndDist().angle / Math.PI * 180));
            telemetry.addData("pole dist",
                    autoStates.getPoleAngleAndDist() == null ? "null" :
                            Double.toString(autoStates.getPoleAngleAndDist().dist));
            telemetry.addData("flag", "HD{W0w_d03S_CoD3_wORK_23eeajfFE32}");
            telemetry.update();
            gp1.update();
        }

        if (task != null) task.cancel();
    }
}
