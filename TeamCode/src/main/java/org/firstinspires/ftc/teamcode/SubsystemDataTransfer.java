package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class SubsystemDataTransfer {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    private static Pose2d CurrentRobotPose;

    public static Pose2d getCurrentRobotPose() { return CurrentRobotPose; }

    public static void setCurrentRobotPose(Pose2d CurrentPose) { CurrentRobotPose = CurrentPose; }

}
