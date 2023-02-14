package org.firstinspires.ftc.teamcode.util.objectdetector;

import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.YELLOW_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isYellowWithMinSaturation;

import android.graphics.Bitmap;

public class  PoleDetector extends DetectorBase {
    // Minimal number of pixel representing the pole width in the image.
    private final static int MIN_POLE_WIDTH = 50;
    private final static double POLE_RADIUS = 0.5;
    private final static String TAG = PoleDetector.class.getSimpleName();

    public static class AngleAndDist {
        public double angle = 0.0;
        public double dist = 1000; // an unrealistic number
    }

    public PoleDetector(CameraFrameSource cameraFrameSource) {
        super(cameraFrameSource);
    }

    /**
     * Returns the view angle of nearby pole relative to the camera in the robot's coordinate. The
     * angle is in radian and positive on the left of the image center vertical line.
     */
    public AngleAndDist getPoleAngleAndDist() {
        AngleAndDist angleAndDist = new AngleAndDist();
        Bitmap bitmap = getFrameBitmap();
        if (bitmap == null) {
            angleAndDist.dist = 1000000; // a large number indicating something is wrong
            return angleAndDist;
        }
        int imageH = bitmap.getHeight();

        // Get the center line with only red, blue and yellow color.
        PixelFilter filter = hsv -> isYellowWithMinSaturation(hsv) ? YELLOW_VALUE : 0;
//        PixelFilter filter = hsv -> isRedWithMinSaturation(hsv) ? RED_VALUE : 0;
        int[] arr = extractCenterLine(bitmap, filter);
        Segment seg = findLongestSegment(arr, MIN_POLE_WIDTH);
        if (seg.length() == 0) {
            arr = extractLineAt(bitmap, imageH / 2 + 100, filter);
            seg = findLongestSegment(arr, MIN_POLE_WIDTH);
            if (seg.length() == 0) {
                arr = extractLineAt(bitmap, imageH / 2 + 200, filter);
                seg = findLongestSegment(arr, MIN_POLE_WIDTH);
            }
        }

        double segLen = seg.length();
        if (segLen < MIN_POLE_WIDTH) return angleAndDist;

        int width = arr.length;
        int offset = seg.midPoint() - width / 2;
        double flipFactor = cameraFrameSource.isUpSideDown() ? -1 : 1;
        double fov = cameraFrameSource.getFOV();
        angleAndDist.angle = offset * fov / width * flipFactor;
        double dist = POLE_RADIUS / (Math.sin(Math.atan(segLen / width * Math.tan(fov * 0.5))));
        // Due to shading effect, some pixels on both side may not be counted as part of the
        // pole after conversion and filtering. Hence, the pole appears smaller than the actual
        // size in the filtered image. Based on the equation, smaller means farther. We need to
        // make the estimated adjustment.
        angleAndDist.dist = dist * 0.98;
        return angleAndDist;
    }
}
