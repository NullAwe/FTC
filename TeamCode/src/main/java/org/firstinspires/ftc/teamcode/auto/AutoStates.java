package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.objectdetector.PoleDetector;

public class AutoStates {

    private int cycleNumber;
    private boolean coneInDelivery;
    private TrajectorySequence currSeq;
    private PoleDetector.AngleAndDist poleAngleAndDist;
    private double curDeliveryOffset;
    private double curPickupOffset;

    public int getCycleNumber() {
        return cycleNumber;
    }

    public boolean isConeInDelivery() {
        return coneInDelivery;
    }

    public TrajectorySequence getCurrSeq() {
        return currSeq;
    }

    public PoleDetector.AngleAndDist getPoleAngleAndDist() {
        return poleAngleAndDist;
    }

    public double getCurDeliveryOffset() {
        return curDeliveryOffset;
    }

    public double getCurPickupOffset() {
        return curPickupOffset;
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

    public void setPoleAngleAndDist(PoleDetector.AngleAndDist poleAngleAndDist) {
        this.poleAngleAndDist = poleAngleAndDist;
    }

    public void setCurDeliveryOffset(double curDeliveryOffset) {
        this.curDeliveryOffset = curDeliveryOffset;
    }

    public void changeCurDeliveryOffset(double delta) {
        this.curDeliveryOffset += delta;
    }

    public void setCurPickupOffset(double curPickupOffset) {
        this.curPickupOffset = curPickupOffset;
    }

    public void changeCurPickupOffset(double delta) {
        this.curPickupOffset += delta;
    }

    public static class Builder {

        private int cycleNumber = 0;
        private boolean coneInDelivery = false;
        private TrajectorySequence currSeq = null;
        private PoleDetector.AngleAndDist poleAngleAndDist = null;
        private double curDeliveryOffset = 0;
        private double curPickupOffset = 0;

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

        public void setPoleAngleAndDist(PoleDetector.AngleAndDist poleAngleAndDist) {
            this.poleAngleAndDist = poleAngleAndDist;
        }

        public void setCurDeliveryOffset(double curDeliveryOffset) {
            this.curDeliveryOffset = curDeliveryOffset;
        }

        public void setCurPickupOffset(double curPickupOffset) {
            this.curPickupOffset = curPickupOffset;
        }

        public AutoStates build() {
            AutoStates states = new AutoStates();
            states.cycleNumber = cycleNumber;
            states.coneInDelivery = coneInDelivery;
            states.currSeq = currSeq;
            states.poleAngleAndDist = poleAngleAndDist;
            states.curDeliveryOffset = curDeliveryOffset;
            states.curPickupOffset = curPickupOffset;
            return states;
        }
    }
}
