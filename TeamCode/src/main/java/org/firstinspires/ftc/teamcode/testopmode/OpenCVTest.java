package org.firstinspires.ftc.teamcode.testopmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@SuppressWarnings({"unused", "RedundantThrows"})
@TeleOp(name="Open CV Test", group="Test")
@Disabled
public class OpenCVTest extends LinearOpMode {

    OpenCvWebcam webcam;
    CustomPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        pipeline = new CustomPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.e("opencvtest", "error on opening camera");
            }
        });

        telemetry.addData("init", "done");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("analysis", pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
        }
    }

    static class CustomPipeline extends OpenCvPipeline {

        enum SignalColor {
            PURPLE,
            GREEN,
            ORANGE
        }

        Mat hsv = new Mat();
        Mat purple = new Mat();
        Mat green = new Mat();
        Mat orange = new Mat();

        SignalColor color = SignalColor.PURPLE;

        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, new Scalar(270, 40, 50), new Scalar(330, 100, 100), purple);
            Core.inRange(hsv, new Scalar(80, 40, 50), new Scalar(150, 100, 100), green);
            Core.inRange(hsv, new Scalar(20, 40, 50), new Scalar(50, 100, 100), orange);
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToHSV(input);
            int p = Core.countNonZero(purple);
            int g = Core.countNonZero(green);
            int o = Core.countNonZero(orange);
            int max = Math.max(p, Math.max(g, o));
            if (max == p) color = SignalColor.PURPLE;
            else if (max == g) color = SignalColor.GREEN;
            else color = SignalColor.ORANGE;
            return input;
        }

        public SignalColor getAnalysis() {
            return color;
        }
    }
}
