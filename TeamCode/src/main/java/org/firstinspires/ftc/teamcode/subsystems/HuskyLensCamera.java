package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;

public class HuskyLensCamera extends SubsystemBase {

    private final ElapsedTime CameraTimer;

    public HuskyLensCamera(HardwareMap map) {
        HuskyLensThreaded huskyLensThread = new HuskyLensThreaded(map);
        SubsystemDataTransfer.HuskyLensThreadActive = true;
        huskyLensThread.start();
        CameraTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void ScanForSample() {
        CameraTimer.reset();
        SubsystemDataTransfer.HuskyLensLoopTime = CameraTimer.time();
    }

}
