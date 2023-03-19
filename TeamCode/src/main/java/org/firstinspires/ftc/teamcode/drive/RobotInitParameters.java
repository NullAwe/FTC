package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotInitParameters {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final RoadRunnerParameters roadRunnerParams;

    public RobotInitParameters(HardwareMap hardwareMap, Telemetry telemetry,
            RoadRunnerParameters roadRunnerParams) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.roadRunnerParams = roadRunnerParams;
    }
}
