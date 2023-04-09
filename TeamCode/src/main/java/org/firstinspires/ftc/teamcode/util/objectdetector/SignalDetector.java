package org.firstinspires.ftc.teamcode.util.objectdetector;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Arrays;
import java.util.List;

public abstract class SignalDetector {
    private TFObjectDetector tfod;

    public void init(HardwareMap hardwareMap, VuforiaLocalizer vuforia, double magnification) {
        initTfod(hardwareMap, vuforia);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(magnification, 16.0/9.0);
        }
    }

    /** Initialize the TensorFlow Object Detection engine. */
    private void initTfod(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        try {
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            loadTFModel(tfod);
        } catch (NullPointerException exception) {
            RobotLog.ee("Initializing TFOD", "got a null pointer exception");
        }
    }

    protected abstract String[] getLabels();

    protected abstract void loadTFModel(TFObjectDetector tfod);

    public int detectSignal() {
        if (tfod == null) return -1;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int labelIndex = -1;
        if (updatedRecognitions != null) {
            // Pick the one with highest confidence
            double confidence = 0.0;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getConfidence() > confidence) {
                    confidence = recognition.getConfidence();
                    // A valid label index is 1-based (1, 2, or 3)
                    labelIndex = Arrays.asList(getLabels()).indexOf(recognition.getLabel()) + 1;
                }
            }
        }
        return labelIndex;
    }

    // Side: 1, 2, 3
    public abstract String getLabel(int side);
}
