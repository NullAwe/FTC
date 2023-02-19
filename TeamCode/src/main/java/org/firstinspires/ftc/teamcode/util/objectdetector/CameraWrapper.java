package org.firstinspires.ftc.teamcode.util.objectdetector;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

public class CameraWrapper implements CameraFrameSource {
    public static final int kImageWidth = 320;
    public static final int kImageHeight = 240;
    private static final String TAG = CameraWrapper.class.getSimpleName();
    private static final int FRAME_QUEUE_CAPACITY = 2;

    /**
     * How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely
     */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    private final WebcamName cameraName;
    private final double fov;
    private final boolean isUpSideDown;
    private CameraManager cameraManager;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    /**
     * The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded.
     */
    private EvictingBlockingQueue<Bitmap> frameQueue;

    /**
     * A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see
     * {@link #startCamera} if you're curious): no knowledge of multi-threading is needed here.
     */
    private Handler callbackHandler;

    public CameraWrapper(WebcamName cameraName, double fov, boolean isUpSideDown) {
        this.cameraName = cameraName;
        this.fov = fov;
        this.isUpSideDown = isUpSideDown;
    }

    @Override
    public Bitmap getFrameBitmap() {
        while (frameQueue.size() > 1) {
            frameQueue.poll();
        }
        return frameQueue.poll();
    }

    @Override
    public double getFOV() {
        return fov;
    }

    @Override
    public boolean isUpSideDown() {
        return isUpSideDown;
    }

    public boolean initCamera() {
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        initializeFrameQueue();

        openCamera();
        if (camera == null) {
            return false;
        }

        startCamera();
        return cameraCaptureSession != null;
    }

    public void stopCamera() {
        if (cameraCaptureSession != null) {
            try {
                cameraCaptureSession.stopCapture();
                cameraCaptureSession.close();
                cameraCaptureSession = null;
            } catch (NullPointerException e) {
                RobotLog.ee(TAG, e,"Null pointer when stopping camera");
            } catch (Exception e) {
                RobotLog.ee(TAG, e, "Exception when stopping camera");
            }
        }
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    /**
     * The frame queue will automatically throw away bitmap frames if they are not processed
     * quickly by the OpMode. This avoids a buildup of frames in memory
     */
    private void initializeFrameQueue() {
        frameQueue =
                new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(FRAME_QUEUE_CAPACITY));
        // not strictly necessary, but helpful
        frameQueue.setEvictAction(Bitmap::recycle);
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            RobotLog.ee(TAG, "camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

         // YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class
         // Definition for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is
         // the only image format supported by a camera
        final int imageFormat = ImageFormat.YUY2;

        // Verify that the image is supported, and fetch size and desired frame rate if so
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            RobotLog.ee(TAG, "Image format not supported");
            return;
        }
        final Size size = new Size(kImageWidth, kImageHeight);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);


         // Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         // here allows us to wait in this method until all that asynchrony completes before
         // returning.
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer =
                new ContinuationSynchronizer<>();
        try {
            // Create a session in which requests to capture frames can be made
            camera.createCaptureSession(Continuation.create(callbackHandler,
                    new CameraCaptureSession.StateCallbackDefault() {
                        @Override
                        public void onConfigured(@NonNull CameraCaptureSession session) {
                            try {
                                // The session is ready to go. Start requesting frames
                                final CameraCaptureRequest captureRequest =
                                        camera.createCaptureRequest(imageFormat, size, fps);
                                session.startCapture(captureRequest,
                                        (session1, request, cameraFrame) -> {
                                            // A new frame is available. The frame data has
                                            // <em>not</em> been copied for us, and we can only
                                            // access it for the duration of the callback. So we
                                            // copy here manually.
                                            Bitmap bmp = captureRequest.createEmptyBitmap();
                                            cameraFrame.copyToBitmap(bmp);
                                            frameQueue.offer(bmp);
                                        },
                                        Continuation.create(callbackHandler, (session12,
                                                cameraCaptureSequenceId, lastFrameNumber) -> RobotLog
                                                .ii(
                                                        TAG,
                                                        "capture sequence %s reports completed: " +
                                                                "lastFrame=%d",
                                                        cameraCaptureSequenceId,
                                                        lastFrameNumber))
                                );
                                synchronizer.finish(session);
                            } catch (CameraException | RuntimeException e) {
                                RobotLog.ee(TAG, e, "exception starting capture");
                                session.close();
                                synchronizer.finish(null);
                            }
                        }
                    }));
        } catch (CameraException | RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            synchronizer.finish(null);
        }

        // Wait for all the async calls to complete
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Retrieve the created session. This will be null on error.
        cameraCaptureSession = synchronizer.getValue();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}
