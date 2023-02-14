package org.firstinspires.ftc.teamcode.util.objectdetector;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaBasedCameraSource implements CameraFrameSource {
    private static final String TAG = CameraWrapper.class.getSimpleName();

    private final VuforiaLocalizer vuforia;
    private final double fov;
    private final boolean isUpSideDown;

    public VuforiaBasedCameraSource(VuforiaLocalizer vuforia, double fov, boolean isUpSideDown) {
        this.vuforia = vuforia;
        this.fov = fov;
        this.isUpSideDown = isUpSideDown;
    }

    @Override
    public Bitmap getFrameBitmap() {
        try {
            if (vuforia != null) {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                for (int i = 0; i < frame.getNumImages(); ++i) {
                    Image image = frame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(),
                                Bitmap.Config.RGB_565);
                        bitmap.copyPixelsFromBuffer(image.getPixels());
                        return bitmap;
                    }
                }
            }
        } catch(InterruptedException e) {
            RobotLog.ee(TAG, "Error on capturing image");
        }
        return null;
    }

    @Override
    public double getFOV() {
        return fov;
    }

    @Override
    public boolean isUpSideDown() {
        return isUpSideDown;
    }
}
