package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemDataTransfer {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    private static Pose2d CurrentRobotPose;

    public static Pose2d getCurrentRobotPose() { return CurrentRobotPose; }

    public static void setCurrentRobotPose(Pose2d CurrentPose) { CurrentRobotPose = CurrentPose; }

    public static boolean OverrideDrivetrainRotation = false;
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PID

    public static boolean IMUWorking = true;

    public static double[] DriveMotorHighCurrents = new double[] {0, 0, 0, 0};

    public static double driveSystemFrameRate = 0;


}
