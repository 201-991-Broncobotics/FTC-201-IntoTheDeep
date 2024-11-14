package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

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
        HuskyLens.Block[] VisionData = SubsystemData.Vision;
        if (VisionData.length >= 1) {

            // find the closest object information
            HuskyLens.Block closestObject = VisionData[0];
            for (HuskyLens.Block value : VisionData) { // if it has the correct color and has the closest distance to the claw
                if (functions.intListContains(SubsystemData.AcceptableIds, value.id) &&
                        Math.hypot(value.x - SubsystemData.HuskyLensTargetX, value.y - SubsystemData.HuskyLensTargetY)
                                < Math.hypot(closestObject.x - SubsystemData.HuskyLensTargetX, closestObject.y - SubsystemData.HuskyLensTargetY)) {
                    closestObject = value; // set that object as the target
                }
            }

            //  return information to arm
            if (functions.intListContains(SubsystemData.AcceptableIds, closestObject.id)) {
                SubsystemData.CameraTargetPixelsX = closestObject.x - SubsystemData.HuskyLensTargetX;
                SubsystemData.CameraTargetPixelsY = -1 * (closestObject.y - SubsystemData.HuskyLensTargetY);
                SubsystemData.CameraTargetId = closestObject.id;
                SubsystemData.CameraSeesValidObject = true;

            } else SubsystemData.CameraSeesValidObject = false;
        } else SubsystemData.CameraSeesValidObject = false;

        SubsystemData.HuskyLensLoopTime = CameraTimer.time();
    }

    public void EndHuskyLensThread() {
        SubsystemData.HuskyLensThreadActive = false;
    }

}
