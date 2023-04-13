package org.firstinspires.ftc.teamcode.task;

import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DELAY_PRIOR_DELIVERY_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DIST_INTAKE_SLIDE_STEP;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.DURATION_INTAKE_SLIDE_DOWN_MILLIS;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_DELIVERY;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.POWER_INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.auto.auto_robot1.AutoBaseRobot1.WAIT_PRIOR_RETRACT_MILLIS;

import org.firstinspires.ftc.teamcode.auto.AutoStates;
import org.firstinspires.ftc.teamcode.common.AngleType;
import org.firstinspires.ftc.teamcode.drive.HDWorldRobotBase;

public class Robot1AutoNormalDeliveryTask implements Task {

    Task task;

    public Robot1AutoNormalDeliveryTask(HDWorldRobotBase robot, AutoStates autoStates) {
        task = new ParallelTask(
                new SeriesTask(
                        new IntakeRotateTask(robot, 0, AngleType.DEGREE),
                        new IntakeSlideTask(robot,
                                (5 - autoStates.getCycleNumber()) * DIST_INTAKE_SLIDE_STEP,
                                POWER_INTAKE_DOWN, DURATION_INTAKE_SLIDE_DOWN_MILLIS),
                        new IntakeClawTask(robot, true)),
                new SeriesTask(
                        new SleepTask(DELAY_PRIOR_DELIVERY_MILLIS),
                        new DeliverySlideTask(robot, robot.getDeliveryHeightHigh(), POWER_DELIVERY),
                        new SleepTask(WAIT_PRIOR_RETRACT_MILLIS)));
    }

    @Override
    public boolean perform() {
        return task.perform();
    }

    @Override
    public void cancel() {
        task.cancel();
    }
}
