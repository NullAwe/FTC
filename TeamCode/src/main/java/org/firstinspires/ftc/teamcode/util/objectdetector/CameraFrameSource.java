package org.firstinspires.ftc.teamcode.util.objectdetector;

import android.graphics.Bitmap;

public interface CameraFrameSource {
    Bitmap getFrameBitmap();

    double getFOV();

    boolean isUpSideDown();
}
