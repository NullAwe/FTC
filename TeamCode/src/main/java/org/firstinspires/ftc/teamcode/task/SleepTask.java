package org.firstinspires.ftc.teamcode.task;

public class SleepTask extends TimedTask {

    public SleepTask(int milliseconds) {
        setFinishTimeMillis(milliseconds);
    }

    @Override
    protected boolean performInternal() {
        return false; // This task is time based, base class logic takes care of it.
    }

    @Override
    public void cancel() {

    }
}
