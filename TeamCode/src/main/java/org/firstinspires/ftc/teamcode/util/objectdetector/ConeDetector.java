package org.firstinspires.ftc.teamcode.util.objectdetector;

import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.BLUE_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.MIN_SATURATION;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.RED_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isBlue;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isRed;

import android.graphics.Bitmap;

public class ConeDetector extends DetectorBase {
    private final static int MIN_CONE_WIDTH = 100;

    public ConeDetector(CameraFrameSource cameraFrameSource) {
        super(cameraFrameSource);
    }

    /**
     * Returns the view angle of nearby cone relative to the camera in the robot's coordinate. The
     * angle is in radian and positive on the left of the image center vertical line.
     */
    public double getConeAngle() {
        Bitmap bitmap = getFrameBitmap();
        if (bitmap == null) return 10000000.0; // Return an unreasonable angle

        // Get the center line with only red, blue and yellow color.
        PixelFilter filter = hsv -> {
            int val = 0;
            if (hsv.s > MIN_SATURATION) {
                if (isRed(hsv)) {
                    val = RED_VALUE;
                } else if (isBlue(hsv)) {
                    val = BLUE_VALUE;
                }
            }
            return val;
        };

        int[] arr = extractCenterLine(bitmap, filter);
        Segment seg = findLongestSegment(arr, MIN_CONE_WIDTH);
        int imageH = bitmap.getHeight();
        if (seg.length() == 0) {
            arr = extractLineAt(bitmap, imageH * 3 / 4, filter);
            seg = findLongestSegment(arr, MIN_CONE_WIDTH);
            if (seg.length() == 0) {
                arr = extractLineAt(bitmap, imageH - 1, filter);
                seg = findLongestSegment(arr, MIN_CONE_WIDTH);
            }
        }

        int segmentMidPoint = seg.midPoint();
        if (segmentMidPoint == 0) return 0.0;

        int offset = segmentMidPoint - arr.length / 2;
        double flipFactor = cameraFrameSource.isUpSideDown() ? -1 : 1;
        return flipFactor * offset * cameraFrameSource.getFOV() / arr.length ;
    }
}
