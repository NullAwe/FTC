package org.firstinspires.ftc.teamcode.util;

public class Utils {
    // Meant to be used as global utils
    private Utils() {}

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }
}
