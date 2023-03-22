package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class DrivingTask implements Task {

    private final HDWorldRobotBase robot;
    private final TrajectorySequence path;
    private boolean first = true;
    private boolean done = false;
    private boolean background = true;

    public DrivingTask(HDWorldRobotBase robot, TrajectorySequence path){
        this(robot, path, false);
    }

    public DrivingTask(HDWorldRobotBase robot, TrajectorySequence path, boolean background){
        this.robot = robot;
        this.path = path;
        this.background = background;
    }

    @Override
    public boolean perform() {
        if (background) {
            return driveAsync();
        }
        return driveSync();
    }

    private boolean driveAsync() {
        if (first) {
            new Thread(() -> drive()).start();
            first = false;
        }
        return done;
    }

    private boolean driveSync() {
        if (first) {
            robot.followTrajectorySequenceAsync(path);
            first = false;
        }

        done = !robot.isBusy();
        if (!Thread.currentThread().isInterrupted() && !done) {
            robot.update();
        }
        return done;
    }

    private void drive() {
        robot.followTrajectorySequence(path);
        done = true;
    }
}
