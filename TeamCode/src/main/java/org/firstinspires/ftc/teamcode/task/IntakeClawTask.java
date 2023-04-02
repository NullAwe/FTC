package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// Speed servo with 90 rpm under 4.8v and 270 degree max
@Config
public class IntakeClawTask extends TimedTask {
    public enum State {
        FULL_OPEN,
        HALF_OPEN,
        CLOSED
    }

    // The value is computed based on Speed servo (gobilda 2000-0025-0003).
    // Max travel: 300 degree
    // RPM: 90RPM for 4.8V and 115RPM for 6V
    // Servo travel DIST between Open and Close: 0.15-0.2 (equivalent to 45-60 degree)
    // Total time = (0.15 or 0.2) * 300 / 360 * 60 / (90 or 115) * 1000
    public static int OPEN_FINISH_TIME_MILLIS = 100;
    public static int HALF_OPEN_FINISH_TIME_MILLIS = 80;
    private static final int CLOSE_FINISH_TIME_MILLIS = 120;

    private final HDWorldRobotBase robot;
    private final State targetState;
    private boolean started = false;

    public IntakeClawTask(HDWorldRobotBase robot, boolean open) {
        this(robot, open ? State.FULL_OPEN : State.CLOSED);
    }

    public IntakeClawTask(HDWorldRobotBase robot, State state) {
        this.robot = robot;
        this.targetState = state;
        switch (targetState) {
            case FULL_OPEN:
                setFinishTimeMillis(OPEN_FINISH_TIME_MILLIS);
                break;
            case HALF_OPEN:
                setFinishTimeMillis(HALF_OPEN_FINISH_TIME_MILLIS);
                break;
            case CLOSED:
            default:
                setFinishTimeMillis(CLOSE_FINISH_TIME_MILLIS);
                break;
        }
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            switch (targetState) {
                case FULL_OPEN:
                    robot.openClaw();
                    break;
                case HALF_OPEN:
                    robot.openClawHalf();
                    break;
                case CLOSED:
                default:
                    robot.closeClaw();
                    break;
            }
        }
        return false;
    }
}
