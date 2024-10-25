package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;

public class HuskyLensCamera extends SubsystemBase {

    private final HuskyLens Camera;

    public HuskyLensCamera(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void ScanForSample() {
        SubsystemDataTransfer.Vision = Camera.blocks();
    }

}
