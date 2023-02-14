package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/*
 * Serial task doesn't honor the unbounded tasks unless it is the last one.
 */
public class SeriesTask implements Task {

    private final List<Task> tasks;
    private int ind;
    private ElapsedTime timer;

    public SeriesTask(Task... tasks) {
        this.tasks = new ArrayList<>();
        this.tasks.addAll(Arrays.asList(tasks));
        ind = 0;
    }

    @Override
    public boolean perform() {
        if (ind == tasks.size()) {
            if (ind > 0 && tasks.get(ind - 1) instanceof UnboundedTask) {
                return tasks.get(ind - 1).perform();
            }
            return true;
        }

        Task currTask = tasks.get(ind);
        if (DebugTask.debugMode && timer == null) {
            timer = new ElapsedTime();
            if (currTask instanceof SeriesTask || currTask instanceof ParallelTask) {
                DebugTask.level++;
            }
        }
        boolean status = currTask.perform();
        if (status) {
            if (DebugTask.debugMode) {
                RobotLog.ee("task timer", "%s - %s - %d",
                        String.join("", Collections.nCopies(6 * DebugTask.level, " ")),
                        currTask.getClass().getSimpleName(), (int)timer.milliseconds());
                timer.reset();
                if (currTask instanceof SeriesTask || currTask instanceof ParallelTask) {
                    DebugTask.level--;
                }
            }
            ind++;
            if (DebugTask.debugMode && ind < tasks.size()) {
                currTask = tasks.get(ind);
                if (currTask instanceof SeriesTask || currTask instanceof ParallelTask) {
                    DebugTask.level++;
                }
            }
        } else if (currTask instanceof ConditionalTask) {
            // If it is a conditional task and return false, we should terminate the rest of tasks.
            ind = tasks.size();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        if (ind < tasks.size()) tasks.get(ind).cancel();
    }

    public SeriesTask add(Task task) {
        tasks.add(task);
        return this;
    }
}
