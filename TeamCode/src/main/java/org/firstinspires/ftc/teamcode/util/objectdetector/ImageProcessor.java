package org.firstinspires.ftc.teamcode.util.objectdetector;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

@Config
public class ImageProcessor {
    public static int MIN_SATURATION = 45;
    public static int MIN_SATURATION_RED = 70;

    public static final int YELLOW_VALUE = 240;
    public static final int RED_VALUE = 160;
    public static final int BLUE_VALUE = 80;

    public static boolean isYellowWithMinSaturation(HSV hsv) {
        return hsv.s > MIN_SATURATION && isYellow(hsv);
    }

    public static boolean isBlueWithMinSaturation(HSV hsv) {
        return hsv.s > MIN_SATURATION && isBlue(hsv);
    }

    public static boolean isRedWithMinSaturation(HSV hsv) {
        return hsv.s > MIN_SATURATION_RED && isRed(hsv);
    }

    public static boolean isBlueOrRedWithMinSaturation(HSV hsv) {
        return hsv.s > MIN_SATURATION && isBlueOrRed(hsv);
    }

    public static boolean isYellow(HSV hsv) {
        return hsv.h > 20 && hsv.h  < 66;
    }

    public static boolean isBlue(HSV hsv) {
        return hsv.h > 200 && hsv.h < 260;
    }

    public static boolean isRed(HSV hsv) {
        return hsv.h < 32 || hsv.h > 340;
    }

    public static boolean isBlueOrRed(HSV hsv) {
        return hsv.h > 200 && hsv.h < 260 || hsv.h < 32 || hsv.h > 340;
    }

    public static HSV ColorToHsv(NormalizedRGBA color) {
        int scale = 256;
        int min = 0;
        int max = 255;
        return rgbToHsv(
                Range.clip((int)(color.red * scale), min, max),
                Range.clip((int)(color.green * scale), min, max),
                Range.clip((int)(color.blue * scale), min, max));
    }

    // Converts color from RGBA format to HSV format
    public static HSV RgbToHsvFast(int color) {
        return rgbToHsv(Color.red(color), Color.green(color), Color.blue(color));
    }

    // Converting RGB to HSV is the most time-consuming operation. It makes up 99% of
    // all computation time for getting the nearest object position. Inlining the code
    // and using integer calculation reduce total time from ~1500 millis to ~500 millis.
    // HSV hsv = ImageProcessor.RgbToHsvFast(newBitmap.getPixel(j, i));
    public static HSV rgbToHsv(int r, int g, int b) {
        HSV hsv = new HSV();
        int cMax = max(r, g, b);
        int cMin = min(r, g, b);
        int diff = cMax - cMin;

        // Finds h:
        if (diff == 0) {
            hsv.h = 0;
        } else if (cMax == r) {
            hsv.h = (60 * (g - b) / diff + 360) % 360;
        } else if (cMax == g) {
            hsv.h = (60 * (b - r) / diff + 120) % 360;
        } else {
            hsv.h = (60 * (r - g) / diff + 240) % 360;
        }

        // Finds s:
        if (cMax == 0) {
            hsv.s = 0;
        } else {
            hsv.s = diff * 100 / cMax;
        }
        // ignore v since we don't use it

        return hsv;
    }

    private static double max(double a, double b, double c) {
        return Math.max(Math.max(a, b), c);
    }

    private static double min(double a, double b, double c) {
        return Math.min(Math.min(a, b), c);
    }

    private static int max(int a, int b, int c) {
        return Math.max(Math.max(a, b), c);
    }

    private static int min(int a, int b, int c) {
        return Math.min(Math.min(a, b), c);
    }
}