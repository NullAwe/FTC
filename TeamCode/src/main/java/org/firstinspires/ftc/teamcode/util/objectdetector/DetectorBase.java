package org.firstinspires.ftc.teamcode.util.objectdetector;

import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.MIN_SATURATION;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.RgbToHsvFast;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isBlue;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isRed;
import static org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor.isYellow;

import android.graphics.Bitmap;
import android.graphics.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@Config
public class DetectorBase {
    public static class Segment {
        public int from;
        public int to; // exclusive

        public Segment(int from, int to) {
            this.from = from;
            this.to = to;
        }

        public int midPoint() {
            return (from + to) / 2;
        }

        public int length() {
            return to - from;
        }
    }
    // -1 means no debugging
    public static int DEBUG_INDEX = -1;
    private static final int DEBUG_PIXEL_SKIP_STEP = 2;
    private static final String TAG = DetectorBase.class.getSimpleName();

    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    protected final CameraFrameSource cameraFrameSource;
    protected boolean cancelRequested = false;

    public DetectorBase(CameraFrameSource cameraFrameSource) {
        this.cameraFrameSource = cameraFrameSource;
    }

    private int[] extractObjects(Bitmap bitmap) {
        if (bitmap == null) return null;

        int[] objectArea = new int[bitmap.getWidth()];
        int i = bitmap.getHeight() / 2;
        for (int j = 0; j < bitmap.getWidth(); j++) {
            if (cancelRequested) return null;

            HSV hsv = RgbToHsvFast(bitmap.getPixel(j, i));
            if (hsv.s < MIN_SATURATION + 1) {
                objectArea[j] = 0;
            } else {
                if (isYellow(hsv)) {
                    objectArea[j] = 1;
                } else if (isRed(hsv)) {
                    objectArea[j] = 2;
                } else if (isBlue(hsv)) {
                    objectArea[j] = 3;
                } else {
                    objectArea[j] = 0;
                }
            }
        }
        return objectArea;
    }

    public void cancel() {
        cancelRequested = true;
    }

    protected void startDetecting() {
        cancelRequested = false;
    }

    protected Bitmap getFrameBitmap() {
        Bitmap bitmap = cameraFrameSource.getFrameBitmap();
        RobotLog.ee(TAG, bitmap == null ? "Failed to get a bitmap" : "Got a bitmap");
        if (bitmap != null && DEBUG_INDEX > -1) {
            saveImage("pole_original.jpg", bitmap);
            if (DEBUG_INDEX == 0) {
                FtcDashboard.getInstance().sendImage(bitmap);
            }
        }
        return bitmap;
    }

    protected int[] extractCenterLine(Bitmap bitmap, PixelFilter filter) {
        if (bitmap == null) {
            return null;
        }
        if (DEBUG_INDEX > 0) {
            createDebugImages(bitmap, filter);
        }
        return extractLineAt(bitmap, bitmap.getHeight() / 2, filter);
    }

    protected int[] extractLineAt(Bitmap bitmap, int at, PixelFilter filter) {
        if (bitmap == null || at < 0 || at > bitmap.getHeight() - 1) return null;

        int[] objectArea = new int[bitmap.getWidth()];
        for (int j = 0; j < bitmap.getWidth(); j++) {
            if (cancelRequested) return null;

            HSV hsv = RgbToHsvFast(bitmap.getPixel(j, at));
            objectArea[j] = filter.filter(hsv);
        }
        return objectArea;
    }

    protected Segment findLongestSegment(int[] arr, int minLen) {
        if (arr == null) return new Segment(0, 0);

        //  StringBuilder builder = new StringBuilder();
        int startIdx = 0;
        int endIdx = arr.length;
        int preStart = 0, preEnd = 0, curStart = -1, curSegmentVal = -1;
        for (int i = startIdx; i < endIdx; ++i) {
        //  builder.append(arr[i] > 0 ? arr[i] : " ");
            if (arr[i] > 0) {
                if (curSegmentVal != arr[i]) {
                    if (curStart >= 0) {
                        if (i - curStart > preEnd - preStart) {
                            preStart = curStart;
                            preEnd = i;
                        }
                    }
                    curSegmentVal = arr[i];
                    curStart = i;
                }
            } else {
                if (curStart >= 0) {
                    if (i - curStart > preEnd - preStart) {
                        preStart = curStart;
                        preEnd = i;
                    }
                    curStart = -1;
                    curSegmentVal = -1;
                }
            }
        }
//        RobotLog.ee("PolePole center line", builder.toString());

        if (curStart >= 0 && endIdx - curStart > preEnd - preStart) {
            preStart = curStart;
            preEnd = endIdx;
        }
        if (preEnd - preStart < minLen) {
            return new Segment(0, 0);
        }
        return new Segment(preStart, preEnd);
    }

    private void createDebugImages(Bitmap bitmap, PixelFilter filter) {
        if (bitmap == null) return;

        int h = bitmap.getHeight();
        int w = bitmap.getWidth();
        int newH = h / DEBUG_PIXEL_SKIP_STEP;
        int newW = w / DEBUG_PIXEL_SKIP_STEP;
        DebugImagesContainer debugImages = new DebugImagesContainer(newH * newW);
        for (int i = 0; i < newH; i ++) {
            for (int j = 0; j < newW; j ++) {
                if (cancelRequested) return;

                HSV hsv = RgbToHsvFast(bitmap.getPixel(j * DEBUG_PIXEL_SKIP_STEP,
                        i * DEBUG_PIXEL_SKIP_STEP));
                int val = filter.filter(hsv);
                int index = i * newW + j;
                int hue = (int) hsv.h;
                debugImages.color1[index] = hue | hue << 8 | hue << 16 | hue << 24;
                debugImages.color2[index] = val | val << 8 | val << 16 | val << 24;
            }
        }

        Bitmap hueImage = Bitmap.createBitmap(debugImages.color1, newW, newH,
                Bitmap.Config.RGB_565);
        saveImage("pole_hue.jpg", hueImage);
        Bitmap objImage = Bitmap.createBitmap(debugImages.color2, newW, newH,
                Bitmap.Config.RGB_565);
        saveImage("pole_obj.jpg", objImage);
        if (DEBUG_INDEX == 1) {
            FtcDashboard.getInstance().sendImage(hueImage);
        } else if (DEBUG_INDEX == 2) {
            FtcDashboard.getInstance().sendImage(objImage);
        }
    }

    private void saveImage(String fileName, Bitmap bitmap) {
        File file = new File(captureDirectory, fileName);
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            }
        } catch (IOException e) {
            RobotLog.ee("ObjectFinder", e, "exception in saveBitmap()");
        }
    }

    private Bitmap rotate90(Bitmap bitmap) {
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix,
                true);
    }

    private Bitmap rotate90CounterClock(Bitmap bitmap) {
        Matrix matrix = new Matrix();
        matrix.postRotate(-90);
        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix,
                true);
    }

}

