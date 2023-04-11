package org.firstinspires.ftc.teamcode.util.objectdetector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.Map;

@Config
public class CustomSignalDetector extends SignalDetector {
    public enum TF_MODEL {
        FRUIT,
        LETTER_WORLD,
        LOGO_WORLD,
        WORLD,
    }

    public static TF_MODEL currentModel = TF_MODEL.LETTER_WORLD;

    private static Map<TF_MODEL, String> modelMap = new HashMap<>();
    private static Map<TF_MODEL, String[]> labelMap = new HashMap<>();

    public CustomSignalDetector() {
        modelMap.put(TF_MODEL.FRUIT,
                "/sdcard/FIRST/tflitemodels/HD_apple_strawberry_banana.tflite");
        modelMap.put(TF_MODEL.LOGO_WORLD,
                "/sdcard/FIRST/tflitemodels/ftc_18225_hd_logo.tflite");
        modelMap.put(TF_MODEL.LETTER_WORLD,
                "/sdcard/FIRST/tflitemodels/ftc_18225_letter.tflite");
        modelMap.put(TF_MODEL.WORLD,
                "/sdcard/FIRST/tflitemodels/ftc_18225_world.tflite");

        labelMap.put(TF_MODEL.FRUIT, new String[]{"1-apple", "2-strawberry", "3-banana"});
        labelMap.put(TF_MODEL.LOGO_WORLD, new String[]{"1-robot", "2-hdlogo", "3-ftclogo"});
        labelMap.put(TF_MODEL.LETTER_WORLD, new String[]{"1-A", "2-bar", "3-bunny"});
        labelMap.put(TF_MODEL.WORLD, new String[]{"1-bunny", "2-logo", "3-robot"});
    }

    @Override
    protected String[] getLabels() {
        return labelMap.get(currentModel);
    }

    @Override
    protected void loadTFModel(TFObjectDetector tfod) {
        tfod.loadModelFromFile(modelMap.get(currentModel), getLabels());
    }

    @Override
    public String getLabel(int index) {
        if (index < 1 || index > 3) return String.format("%d invalid index", index);
        return getLabels()[index - 1];
    }
}
