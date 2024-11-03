package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    public static Pose2d CurrentRobotPose;

    public static boolean OverrideDrivetrainRotation = false;
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PID

    public static boolean IMUWorking = true;

    public static double[] DriveMotorHighCurrents = new double[] {0, 0, 0, 0};

    public static HuskyLens.Block[] Vision;

    public static double HuskyLensLoopTime = 0, HuskyLensThreadLoopTime = 0, DrivetrainLoopTime = 0, IMUThreadTime = 0;
    public static boolean HuskyLensThreadActive = false, IMUThreadActive = false;

    public static GamepadEx operator, driver;

    public static DcMotorEx brokenDiffyEncoder;

    public static boolean HuskyLensConnected = false;

    public static YawPitchRollAngles IMUAngles;

    public static AngularVelocity IMUAngularVelocity;

    public static double SwerveModuleKp, SwerveModuleKi, SwerveModuleKd;


}
