package org.firstinspires.ftc.teamcode.util.objectdetector;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class DefaultSignalDetector extends SignalDetector {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {"1 Bolt", "2 Bulb", "3 Panel"};

    @Override
    protected String[] getLabels() {
        return LABELS;
    }

    @Override
    protected void loadTFModel(TFObjectDetector tfod) {
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public String getLabel(int index) {
        if (index < 1 || index > 3) return String.format("%d invalid index", index);
        return LABELS[index - 1];
    }
}
