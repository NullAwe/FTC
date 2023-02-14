package org.firstinspires.ftc.teamcode.util.objectdetector;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public interface CameraProvider {
    WebcamName[] getCameraNames();
    WebcamName getArmCamera();
    WebcamName getClawCamera();
    double getClawCameraFov();
    boolean isClawCameraUpSideDown();
}
