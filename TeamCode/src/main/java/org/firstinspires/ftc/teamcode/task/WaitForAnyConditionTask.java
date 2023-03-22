package org.firstinspires.ftc.teamcode.task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Serial task doesn't honor the unbounded tasks unless it is the last one.
 */
public class WaitForAnyConditionTask implements Task {

    private final List<Task> tasks;
    private int ind;

    public WaitForAnyConditionTask(Task... tasks) {
        this.tasks = new ArrayList<>();
        this.tasks.addAll(Arrays.asList(tasks));
    }

    @Override
    public boolean perform() {
        boolean done = false;
        for (Task task : tasks) {
            if (task.perform()) {
                done = true;
                break;
            }
        }
        if (done) {
            for (Task task : tasks) {
                task.cancel();
            }
        }
        return done;
    }

    public WaitForAnyConditionTask add(Task task) {
        tasks.add(task);
        return this;
    }
}
