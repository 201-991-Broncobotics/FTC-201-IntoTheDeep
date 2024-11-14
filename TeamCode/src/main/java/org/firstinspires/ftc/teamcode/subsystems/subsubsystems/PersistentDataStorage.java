package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

// A static field allows data to persist between OpModes.

import com.acmerobotics.roadrunner.Pose2d;

public class PersistentDataStorage {
    public static Pose2d lastRobotPose = new Pose2d(0, 0, Math.toRadians(90));

    public static double lastLeftDiffyAngle = 0, lastRightDiffyAngle = 0;

}