package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubsystemData;

public class HuskyLensCamera extends SubsystemBase {

    private final ElapsedTime CameraTimer;
    HuskyLensThreaded huskyLensThread;

    public HuskyLensCamera(HardwareMap map) {
        huskyLensThread = new HuskyLensThreaded(map);
        CameraTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void StartHuskyLensThread() {
        SubsystemData.HuskyLensThreadActive = true;
        huskyLensThread.start();
    }

    public void ScanForSample() {
        CameraTimer.reset();

        SubsystemData.HuskyLensLoopTime = CameraTimer.time();
    }

    public void EndHuskyLensThread() {
        SubsystemData.HuskyLensThreadActive = false;
    }

}
