package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TimedTask implements Task {

    private int finishTimeMillis = 10000;

    protected ElapsedTime timer = null;

    public boolean perform() {
        if (timer == null) timer = new ElapsedTime();
        if (timer.milliseconds() > finishTimeMillis) {
            cancel();
            return true;
        }
        return performInternal();
    }

    protected abstract boolean performInternal();

    protected void setFinishTimeMillis(int finishTimeMillis) {
        this.finishTimeMillis = finishTimeMillis;
    }
}
