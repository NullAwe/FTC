package org.firstinspires.ftc.teamcode.task;

public class WaitTask extends TimedTask {

    public WaitTask(int millis) { setFinishTimeMillis(millis); }

    @Override
    protected boolean performInternal() {
        return false;
    }
}
