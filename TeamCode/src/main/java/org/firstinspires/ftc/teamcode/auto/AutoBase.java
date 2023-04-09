package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.task.SignalDetectionTask;
import org.firstinspires.ftc.teamcode.task.Task;

public abstract class AutoBase extends LinearOpMode {
    public static int AA_NUM_CYCLES = 5;
    public static int AA_TOTAL_TIME_MILLIS = 30000;
    public static int parkingZone = 2; // 1, 2, 3

    protected final ElapsedTime timer = new ElapsedTime();
    protected HDWorldRobotBase robot;
    protected AutoState state = AutoState.INIT;
    protected int cycleNumber = 0;
    protected TrajectorySequence prevSeq;
    protected TrajectorySequence currSeq;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap, telemetry);
        resetRobot();
        Task task = createStartTask();

        // Signal detection is running in a background thread to avoid blocking the main thread. It
        // is important since robot, otherwise, might not react right away when pressing the START
        // button. Hence, it will waist 1-2 seconds depending on the timing.
        SignalDetectionTask signalDetectionTask = new SignalDetectionTask(robot);
        signalDetectionTask.startDetection();
        while (!isStopRequested() && !isStarted()) {
            if (signalDetectionTask.perform()) {
                int newPos = signalDetectionTask.getRecognizedSignal();
                if (newPos >= 1 && newPos <= 3) parkingZone = newPos;
                if (isStarted()) break;
                signalDetectionTask.startDetection();
                telemetry.addData("Signal", "position: %s", robot.getSignalLabel(parkingZone));
                telemetry.update();
            }
        }
        if (parkingZone < 1 || parkingZone > 3) parkingZone = 2;

        timer.reset();
        boolean done = false;
        while (opModeIsActive() && !done && task != null) {
            robot.updateEncoderValues();
            if (timeToFinish() && state != AutoState.FINISH) {
                RobotLog.ee("Finish", "not enough time, quitting at %f", timer.milliseconds());
                task.cancel();
                task = createFinishTask();
            } else if (task.perform()) {
                if (hasTimeForOneMoreCycle()) {
                    cycleNumber++;
                    task = createCycleTask();
                } else if (state != AutoState.FINISH) {
                    task = createFinishTask();
                } else {
                    done = true;
                }
            }
            telemetry.addData("status", "running");
            telemetry.update();
        }
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - 1500;
    }

    private boolean hasTimeForOneMoreCycle() {
        // Go ahead if there are still 6 seconds left and .
        return cycleNumber < AA_NUM_CYCLES && timer.milliseconds() < AA_TOTAL_TIME_MILLIS - 6000;
    }

    private void resetRobot() {
        robot.setIntakeRotateAngle(0);
        robot.setDeliveryRotateAngle(0);
        robot.closeClaw();
        robot.initConeRighter();
    }

    protected abstract Task createStartTask();

    protected abstract Task getDeliveryTask();

    protected abstract Task createFinishTask();

    protected abstract Task createCycleTask();

    protected int getSign() {
        return isBlueCorner() ? -1 : 1;
    }

    protected abstract boolean isBlueCorner();

    protected abstract String getTeleOpName();

    protected abstract HDWorldRobotBase createRobot(HardwareMap hardwareMap, Telemetry telemetry);

    protected enum AutoState {
        INIT,
        FINISH,      // Trying to finish and park
        CYCLE,
    }
}
