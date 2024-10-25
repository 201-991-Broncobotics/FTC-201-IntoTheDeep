package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;

public class HuskyLensCamera extends SubsystemBase {

    private final HuskyLens Camera;

    private ElapsedTime CameraTimer;

    public HuskyLensCamera(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        CameraTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void ScanForSample() {
        CameraTimer.reset();
        SubsystemDataTransfer.Vision = Camera.blocks();
        SubsystemDataTransfer.HuskyLensLoopTime = CameraTimer.time();
    }

}
