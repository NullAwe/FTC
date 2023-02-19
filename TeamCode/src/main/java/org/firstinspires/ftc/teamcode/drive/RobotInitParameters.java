package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotInitParameters {
    public final HardwareMap hardwareMap;
    public final RoadRunnerParameters roadRunnerParams;

    public RobotInitParameters(HardwareMap hardwareMap, RoadRunnerParameters roadRunnerParams) {
        this.hardwareMap = hardwareMap;
        this.roadRunnerParams = roadRunnerParams;
    }
}
