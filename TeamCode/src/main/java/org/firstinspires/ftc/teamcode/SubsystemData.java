package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

import java.util.ArrayList;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    public static Pose2d CurrentRobotPose;

    public static boolean OverrideDrivetrainRotation = false;
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PID

    public static boolean IMUWorking;
    public static IMU imuInstance;

    public static double[] DriveMotorHighCurrents = new double[] {0, 0, 0, 0};

    public static HuskyLens.Block[] Vision;

    public static double CameraTargetPixelsX, CameraTargetPixelsY;

    public static int[] AcceptableIds = new int[] {1, 2, 3}; // 1 = red, 2 = yellow, 3 = blue

    public static double HuskyLensTargetX = 160, HuskyLensTargetY = 190; // huskylens screen is 320 pixels width (x) and 240 pixels height (y)
    // note that 0,0 on the huskylens is the top left corner

    public static boolean CameraSeesValidObject = false;

    public static int CameraTargetId = 0;

    public static double HuskyLensLoopTime = 0, HuskyLensThreadLoopTime = 0, DrivetrainLoopTime = 0, IMUThreadTime = 0;
    public static boolean HuskyLensThreadActive = false, IMUThreadActive = false;

    public static GamepadEx operator, driver;

    public static DcMotorEx brokenDiffyEncoder;

    public static boolean HuskyLensConnected = false;

    public static YawPitchRollAngles IMUAngles;

    public static AngularVelocity IMUAngularVelocity;

    public static double SwerveModuleKp, SwerveModuleKi, SwerveModuleKd;

    public static boolean needToResetModuleEncoders = true;

    public static double RRVoltage = 0;

    public static double RRMaxVoltage = 0;

    public static boolean HoldClawFieldPos;


    public static double OperatorTurningPower = 0;
    public static double AutoAimHeading = 0;

    public static double HeadingHold = 0;


}
