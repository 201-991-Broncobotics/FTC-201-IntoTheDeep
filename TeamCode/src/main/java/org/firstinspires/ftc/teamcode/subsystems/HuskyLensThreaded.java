package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;

public class HuskyLensThreaded extends Thread {

    private final HuskyLens Camera;

    private final ElapsedTime ThreadTimer;

    @Override
    public void run() {
        while (SubsystemDataTransfer.HuskyLensThreadActive) {
            ThreadTimer.reset();
            SubsystemDataTransfer.Vision = Camera.blocks();
            SubsystemDataTransfer.HuskyLensLoopTime = ThreadTimer.time();
        }
    }

    public HuskyLensThreaded(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        ThreadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        SubsystemDataTransfer.Vision = Camera.blocks();
    }

}
