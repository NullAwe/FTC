package org.firstinspires.ftc.teamcode.util.objectdetector;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class CustomSignalDetector extends SignalDetector {
    private static final String TFOD_MODEL_FILE_FRUIT =
            "/sdcard/FIRST/tflitemodels/HD_apple_strawberry_banana.tflite";
    private static final String TFOD_MODEL_FILE_LOGO =
            "/sdcard/FIRST/tflitemodels/hd_logo.tflite";
    private static final String TFOD_MODEL_FILE_FRUIT_WORLDS =
            "/sdcard/FIRST/tflitemodels/ftc_18225_fruit.tflite";
    private static final String TFOD_MODEL_FILE_SHAPE_WORLDS =
            "/sdcard/FIRST/tflitemodels/ftc_18225_shape.tflite";
    private static final String TFOD_MODEL_FILE_LETTER_WORLDS =
            "/sdcard/FIRST/tflitemodels/ftc_18225_letter.tflite";
    private static final String[] FRUIT_LABELS = {"1-apple", "2-strawberry", "3-banana"};
    private static final String[] LOGO_LABELS = {"1-robot", "2-hdlogo", "3-ftclogo"};
    private static final String[] FRUIT_WORLDS_LABELS = {"1-apple", "2-banana", "3-grape"};
    private static final String[] SHAPE_WORLDS_LABELS =
            {"1-yellow", "2-checkerboard","3-triangle"};
    private static final String[] LETTER_WORLDS_LABELS = {"1-A", "2-bar","3-bunny"};


    @Override
    protected String[] getLabels() {
        return FRUIT_LABELS;
//        return LOGO_LABELS;
    }

    @Override
    protected void loadTFModel(TFObjectDetector tfod) {
        tfod.loadModelFromFile(TFOD_MODEL_FILE_FRUIT, FRUIT_LABELS);
//        tfod.loadModelFromFile(TFOD_MODEL_FILE_LOGO, LOGO_LABELS);
    }
}
