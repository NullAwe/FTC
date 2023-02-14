package org.firstinspires.ftc.teamcode.util.objectdetector;

import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.BLUE_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.MIN_SATURATION;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.RED_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.YELLOW_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isBlue;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isRed;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isYellow;

import android.graphics.Bitmap;

public class CameraDebugger extends DetectorBase {
    public CameraDebugger(CameraFrameSource frameSource) {
        super(frameSource);
    }

    public Segment showFrame() {
        if (DEBUG_INDEX < 0 || DEBUG_INDEX > 2) {
            return new Segment(0, 0);
        }

        Bitmap bitmap = getFrameBitmap();

        int[] arr = extractCenterLine(bitmap, hsv -> {
            int val = 0;
            if (hsv.s > MIN_SATURATION) {
                if (isYellow(hsv)) val = YELLOW_VALUE;
                else if (isBlue(hsv)) val = BLUE_VALUE;
                else if (isRed(hsv)) val = RED_VALUE;
            }
            return val;
        });

        if (arr == null) return new Segment(0, 0);

        return findLongestSegment(arr, 1);
    }
}
