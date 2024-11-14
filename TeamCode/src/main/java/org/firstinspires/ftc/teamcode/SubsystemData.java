package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    public static Pose2d CurrentRobotPose;

    public static boolean absoluteDriving;

    public static boolean OverrideDrivetrainRotation = false;
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID, AxialPID, LateralPID; // temporarily here so I can tune the PID

    public static Pose2d AutonError = new Pose2d(new Vector2d(0, 0), 0);

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

    public static double HuskyLensLoopTime = 0, HuskyLensThreadLoopTime = 0, DrivetrainLoopTime = 0;
    public static boolean HuskyLensThreadActive = false;

    public static GamepadEx operator, driver;

    public static DcMotorEx brokenDiffyEncoder;

    public static boolean HuskyLensConnected = false;

    public static YawPitchRollAngles IMUAngles;

    public static double SwerveModuleKp = 0.005, SwerveModuleKi = 0.0, SwerveModuleKd = 0.0;

    public static int SwerveModuleDriveSharpness = 1; // 1 is normal, a value higher than 1 would make the module have much less power when not at the correct angle

    public static boolean HoldClawFieldPos;


    public static double OperatorTurningPower = 0;
    public static double AutoAimHeading = 0;

    public static boolean NeedToRealignHeadingHold = false;


    public static int CommandBlendingAmount = 1;


    // Swerve brake waddle
    public static double SwitchTimeMS = 400, SwitchTimeTimeout = 750; // not including time to rotate module
    public static double SwerveModuleTolerance = 8; // degrees
    public static double lowerTriggerThreshold = 0.25, upperTriggerThreshold = 0.8; // lower is when waddle starts, upper is when just full brake
    public static double maxBrakeWaddleAngle = 40;


}
