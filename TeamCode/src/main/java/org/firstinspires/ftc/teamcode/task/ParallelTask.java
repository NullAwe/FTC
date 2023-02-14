package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/*
 * Parallel task honor unbounded tasks.
 */
public class ParallelTask implements Task {

    private final Set<Task> tasks;
    private ElapsedTime timer;

    public ParallelTask(Task... tasks) {
        this.tasks = new HashSet<>(Arrays.asList(tasks));
    }

    @Override
    public boolean perform() {
        if (DebugTask.debugMode && timer == null) {
            timer = new ElapsedTime();
        }
        Set<Task> remove = new HashSet<>();
        int finishedUnboundedTasks = 0;
        for (Task task : tasks) {
            if (task.perform()) {
                if (task instanceof UnboundedTask) {
                    finishedUnboundedTasks++;
                } else {
                    if (DebugTask.debugMode) {
                        RobotLog.ee("task timer", "%s - %s (parallel) - %d",
                                String.join("", Collections.nCopies(6 * DebugTask.level, " ")),
                                task.getClass().getSimpleName(), (int)timer.milliseconds());
                    }
                    remove.add(task);
                }
            }
        }
        for (Task task : remove) tasks.remove(task);
        if (tasks.size() == finishedUnboundedTasks) {
            if (DebugTask.debugMode) {
                for (Task task : tasks) {
                    RobotLog.ee("task timer", "%s - %s (unbound, parallel) - %d",
                            String.join("", Collections.nCopies(6 * DebugTask.level, " ")),
                            task.getClass().getSimpleName(), (int) timer.milliseconds());
                }
            }
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        for (Task task : tasks) {
            task.cancel();
        }
    }

    public ParallelTask add(Task task) {
        tasks.add(task);
        return this;
    }
}
