package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubsystemData;

public class HuskyLensThreaded extends Thread {

    private final HuskyLens Camera;

    private final ElapsedTime ThreadTimer;

    @Override
    public void run() {
        while (SubsystemData.HuskyLensThreadActive) {
            ThreadTimer.reset();
            SubsystemData.Vision = Camera.blocks();
            SubsystemData.HuskyLensConnected = Camera.knock();
            SubsystemData.HuskyLensThreadLoopTime = ThreadTimer.time();
        }
        SubsystemData.CameraSeesValidObject = false;
    }

    public HuskyLensThreaded(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        ThreadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        SubsystemData.HuskyLensConnected = Camera.knock();
        SubsystemData.Vision = Camera.blocks();
    }

}
