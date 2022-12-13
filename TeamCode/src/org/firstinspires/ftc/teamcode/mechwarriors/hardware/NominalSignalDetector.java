package org.firstinspires.ftc.teamcode.mechwarriors.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;

public class NominalSignalDetector {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public NominalSignalDetector(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public SignalSide detect() {
        return SignalSide.NONE;
    }
}