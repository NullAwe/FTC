package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.util.drive_algorithms.DriveWithPID;

// Speed servo with 90 rpm under 4.8v and 270 degree max
public class DriveStraightTask implements Task {
    private final DriveWithPID driver;
    private final double dist;

    public DriveStraightTask(HDWorldRobotBase robot, double dist) {
        driver = new DriveWithPID(robot);
        this.dist = dist;
    }

    @Override
    public boolean perform() {
        return driver.move(dist);
    }
}
