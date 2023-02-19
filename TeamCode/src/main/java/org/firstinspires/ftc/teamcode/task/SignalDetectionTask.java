package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// SignalDetectionTask the run signal detection in the background and can be restarted again and
// again.
public class SignalDetectionTask implements Task {
    private final HDWorldRobotBase robot;
    private int recognizedSignal = -1;
    private State state = State.INIT;

    public SignalDetectionTask(HDWorldRobotBase robot) {
        this.robot = robot;
    }

    public void startDetection() {
        if (state == State.DETECTING) return;

        state = State.DETECTING;
        new Thread(() -> {
            recognizedSignal = robot.detectSignal();
            state = State.DONE;
        }).start();
    }

    @Override
    public boolean perform() {
        return state == State.DONE;
    }

    public int getRecognizedSignal() {
        return recognizedSignal;
    }

    enum State {
        INIT,
        DETECTING,
        DONE
    }
}
