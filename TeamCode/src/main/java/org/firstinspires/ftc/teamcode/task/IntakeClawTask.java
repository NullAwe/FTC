package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

// Speed servo with 90 rpm under 4.8v and 270 degree max
public class IntakeClawTask extends TimedTask {

    // The value is computed based on Speed servo (gobilda 2000-0025-0003).
    // Max travel: 300 degree
    // RPM: 90RPM for 4.8V and 115RPM for 6V
    // Servo travel DIST between Open and Close: 0.15-0.2 (equivalent to 45-60 degree)
    // Total time = (0.15 or 0.2) * 300 / 360 * 60 / (90 or 115) * 1000
    private static final int FINISH_TIME_MILLIS = 110;

    private final HDWorldRobotBase robot;
    private final boolean open;

    public IntakeClawTask(HDWorldRobotBase robot, boolean open) {
        this.robot = robot;
        this.open = open;
        setFinishTimeMillis(FINISH_TIME_MILLIS);
    }
    @Override
    protected boolean performInternal() {
        if (open) {
            robot.openClaw();
        } else {
            robot.closeClaw();
        }
        return false;
    }
}
