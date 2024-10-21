package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HuskyLensCamera {

    private final HuskyLens Camera;

    public HuskyLensCamera(HardwareMap map) {
        Camera = map.get(HuskyLens.class, "HuskyLens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void ScanForSample(Telemetry telemetry) {
        HuskyLens.Block[] block = Camera.blocks();
        telemetry.addData("HuskyLens block count:", block.length);
        for (HuskyLens.Block value : block) {
            telemetry.addLine("ID:" + (value.id) + " x:" + (value.x) + " y:" + (value.y) + // Id, center X, center Y
                    " h:" + (value.height) + " w:" + (value.width)); // height, width,  + " ox" + (value.left) + " oy" + (value.top)  origin X, Origin Y
        }
    }

}
