package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Settings;
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
                if (functions.intListContains(Settings.AcceptableIds, value.id) &&
                        Math.hypot(value.x - Settings.HuskyLensTargetX, value.y - Settings.HuskyLensTargetY)
                                < Math.hypot(closestObject.x - Settings.HuskyLensTargetX, closestObject.y - Settings.HuskyLensTargetY)) {
                    closestObject = value; // set that object as the target
                }
            }


            //  return information to arm
            if (functions.intListContains(Settings.AcceptableIds, closestObject.id)) {
                SubsystemData.CameraTargetPixelsX = closestObject.x - Settings.HuskyLensTargetX;
                SubsystemData.CameraTargetPixelsY = -1 * (closestObject.y - Settings.HuskyLensTargetY);
                SubsystemData.CameraTargetsPixelsWidth = closestObject.width;
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
