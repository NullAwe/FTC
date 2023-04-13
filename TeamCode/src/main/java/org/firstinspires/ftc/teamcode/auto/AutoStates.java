package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class AutoStates {

    private int cycleNumber;
    private boolean coneInDelivery;
    private TrajectorySequence currSeq;

    public int getCycleNumber() {
        return cycleNumber;
    }

    public boolean isConeInDelivery() {
        return coneInDelivery;
    }

    public TrajectorySequence getCurrSeq() {
        return currSeq;
    }

    public void setCycleNumber(int cycleNumber) {
        this.cycleNumber = cycleNumber;
    }

    public void setConeInDelivery(boolean coneInDelivery) {
        this.coneInDelivery = coneInDelivery;
    }

    public void setCurrSeq(TrajectorySequence currSeq) {
        this.currSeq = currSeq;
    }

    public static class Builder {

        private int cycleNumber = 0;
        private boolean coneInDelivery = false;
        private TrajectorySequence currSeq = null;

        public Builder setCycleNumber(int cycleNumber) {
            this.cycleNumber = cycleNumber;
            return this;
        }

        public Builder setConeInDelivery(boolean coneInDelivery) {
            this.coneInDelivery = coneInDelivery;
            return this;
        }


        public void setCurrSeq(TrajectorySequence currSeq) {
            this.currSeq = currSeq;
        }

        public AutoStates build() {
            AutoStates states = new AutoStates();
            states.cycleNumber = cycleNumber;
            states.coneInDelivery = coneInDelivery;
            states.currSeq = currSeq;
            return states;
        }
    }
}
