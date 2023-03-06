package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.util.drive_algorithms.DriveWithPID;

// Speed servo with 90 rpm under 4.8v and 270 degree max
public class DriveTurnTask implements Task {
    private final DriveWithPID driver;
    private final int angleDegree;

    public DriveTurnTask(HDWorldRobotBase robot, int angleDegree) {
        driver = new DriveWithPID(robot);
        this.angleDegree = angleDegree;
    }

    @Override
    public boolean perform() {
        return driver.rotate(angleDegree);
    }
}
