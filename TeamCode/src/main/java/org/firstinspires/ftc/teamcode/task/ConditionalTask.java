package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

/**
 * A task tells if the operation should continue or terminate. This should only be used in a
 * SeriesTask.
 */
public class ConditionalTask implements Task {
    private final Condition condition;

    public ConditionalTask(Condition condition) {
        this.condition = condition;
    }

    @Override
    public boolean perform() {
        return condition.shouldContinue();
    }
}
