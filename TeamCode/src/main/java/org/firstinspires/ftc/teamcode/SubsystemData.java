package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    private static Pose2d CurrentRobotPose;

    public static Pose2d getCurrentRobotPose() { return CurrentRobotPose; } // TODO: why did I do this for only robot pose????

    public static void setCurrentRobotPose(Pose2d CurrentPose) { CurrentRobotPose = CurrentPose; }

    public static boolean OverrideDrivetrainRotation = false;
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PID

    public static boolean IMUWorking = true;

    public static double[] DriveMotorHighCurrents = new double[] {0, 0, 0, 0};

    public static HuskyLens.Block[] Vision;

    public static double HuskyLensLoopTime = 0, HuskyLensThreadLoopTime = 0, DrivetrainLoopTime = 0;
    public static boolean HuskyLensThreadActive = false;

    public static LazyImu imuChecker;

    public static GamepadEx operator, driver;


}
