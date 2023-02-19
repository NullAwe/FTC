package org.firstinspires.ftc.teamcode.util.objectdetector;

import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.YELLOW_VALUE;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isBlueOrRedWithMinSaturation;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isYellowWithMinSaturation;

import android.graphics.Bitmap;

public class ConeStackDetector extends DetectorBase {
    // Minimal number of pixel representing the cone width in the image.
    private final static int MIN_CONE_WIDTH = 50;
    private final static double CONE_RADIUS = 0.5;
    private final static String TAG = ConeStackDetector.class.getSimpleName();

    public static class AngleAndDist {
        public double angle = 0.0;
        public double dist = 1000; // an unrealistic number
    }

    public ConeStackDetector(CameraFrameSource cameraFrameSource) {
        super(cameraFrameSource);
    }

    /**
     * Returns the view angle of nearby pole relative to the camera in the robot's coordinate. The
     * angle is in radian and positive on the left of the image center vertical line.
     * Steps: (
     *  1. Define a height range
     *  2. Find the middle horizontal segment with red/blue/yellow color -> width range (skip 1)
     *  3. Bottom to up -> find the widest segment (skip 1 in both directions)
     *  4. Get the height of the cone stack
     *  5. Get the
     */
    public AngleAndDist getColeStackAngleAndDist() {
        AngleAndDist angleAndDist = new AngleAndDist();
        Bitmap bitmap = getFrameBitmap();
        if (bitmap == null) {
            angleAndDist.dist = 1000000; // a large number indicating something is wrong
            return angleAndDist;
        }
        int imageH = bitmap.getHeight();
        int imageW = bitmap.getWidth();

        // Get the center line with only red, blue and yellow color.
        PixelFilter filter = hsv -> isBlueOrRedWithMinSaturation(hsv) ? YELLOW_VALUE : 0;
        int[] arr = extractCenterLine(bitmap, filter);
        Segment seg = findLongestSegment(arr, MIN_CONE_WIDTH);
        double segLen = seg.length();
        if (segLen < MIN_CONE_WIDTH) return angleAndDist;

        int width = arr.length;
        int offset = seg.midPoint() - width / 2;
        double flipFactor = cameraFrameSource.isUpSideDown() ? -1 : 1;
        double fov = cameraFrameSource.getFOV();
        angleAndDist.angle = offset * fov / width * flipFactor;
        double dist = CONE_RADIUS / (Math.sin(Math.atan(segLen / width * Math.tan(fov * 0.5))));
        // Due to shading effect, some pixels on both side may not be counted as part of the
        // pole after conversion and filtering. Hence, the pole appears smaller than the actual
        // size in the filtered image. Based on the equation, smaller means farther. We need to
        // make the estimated adjustment.
        angleAndDist.dist = dist * 0.98;
        return angleAndDist;
    }
}
