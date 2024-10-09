package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class HuskyLensCamera {

    private HuskyLens Camera;

    public HuskyLensCamera(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void ScanForSample() {

    }

}
