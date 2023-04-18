package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

public class CheckConditionalTask implements Task {
    private final Condition condition;
    private final Task task;
    private boolean checked = false;

    public CheckConditionalTask(Condition condition, Task task) {
        this.condition = condition;
        this.task = task;
    }

    @Override
    public boolean perform() {
        if (!checked && !condition.shouldContinue()) return true;
        else checked = true;
        return task.perform();
    }
}
