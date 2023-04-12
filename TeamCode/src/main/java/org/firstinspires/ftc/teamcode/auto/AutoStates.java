package org.firstinspires.ftc.teamcode.auto;

public class AutoStates {

    private int cycleNumber;
    private boolean coneInDelivery;

    public int getCycleNumber() {
        return cycleNumber;
    }

    public boolean isConeInDelivery() {
        return coneInDelivery;
    }

    public void setCycleNumber(int cycleNumber) {
        this.cycleNumber = cycleNumber;
    }

    public void setConeInDelivery(boolean coneInDelivery) {
        this.coneInDelivery = coneInDelivery;
    }

    public static class Builder {

        private int cycleNumber;
        private boolean coneInDelivery;

        public Builder setCycleNumber(int cycleNumber) {
            this.cycleNumber = cycleNumber;
            return this;
        }

        public Builder setConeInDelivery(boolean coneInDelivery) {
            this.coneInDelivery = coneInDelivery;
            return this;
        }

        public AutoStates build() {
            AutoStates states = new AutoStates();
            states.cycleNumber = cycleNumber;
            states.coneInDelivery = coneInDelivery;
            return states;
        }
    }
}
