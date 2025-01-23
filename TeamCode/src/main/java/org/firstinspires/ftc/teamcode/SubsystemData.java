package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    public static Pose2d CurrentRobotPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90)); // from roadrunner
    public static Pose CurrentPedroPose = new Pose(0, 0, Math.toRadians(90)); // from Pedro Pathing
    public static Vector CurrentPedroVelocity;

    public static double FrameRate = 20; // just starting value, it is constantly updated in ArmSystem


    public static boolean inTeleOp;

    public static boolean absoluteDriving;

    public static boolean OverrideDrivetrainRotation = false; // for auto aiming
    public static double OverrideDrivetrainTargetHeading = 0;

    public static Pose2d AutonError = new Pose2d(new Vector2d(0, 0), 0);

    public static boolean IMUWorking;
    public static IMU imuInstance;
    public static boolean needToResetIMU = false;


    public static HuskyLens.Block[] Vision;
    public static double CameraTargetPixelsX, CameraTargetPixelsY, CameraTargetsPixelsWidth;
    public static boolean CameraSeesValidObject = false;
    public static int CameraTargetId = 0;

    public static double HuskyLensLoopTime = 0, HuskyLensThreadLoopTime = 0, DrivetrainLoopTime = 0;
    public static boolean HuskyLensThreadActive = false;

    public static GamepadEx operator, driver;

    public static DcMotorEx brokenDiffyEncoder;
    public static boolean HuskyLensConnected = false;
    public static YawPitchRollAngles IMUAngles;


    // This doesn't doing anything but lets me change this ones variables in PID Tuner and both swerve modules will copy the same settings from this one
    public static PIDController SwerveModuleReferencePID = new PIDController(0.01, 0.0, 0.0003, () -> 0);
    public static boolean HoldClawFieldPos = false;
    public static double OperatorTurningPower = 0;
    public static double HeadHoldTarget = 0;
    public static boolean NeedToRealignHeadingHold = false;
    public static boolean repeatForwardBack = false;
    public static double HighDriveVel = 0, HighAngVel = 0, HighDriveAccel = 0, LowDriveAccel = 0, HighAngAccel = 0;
    public static boolean alreadyAlignedArm = false;
    public static PoseVelocity2d RobotVelocity; // constantly updates with localizer instead of requiring updating localizer an extra time

    public static double DriveCurrentExtensionLengthPercent = 0;

    public static Pose TargetPedroPose = new Pose();

    public static boolean AutoDriving = false;
    public static int CurrentPathSetting = 0; // Submersible, Human Player, Chamber, Basket

    public static double AutoDrivingPower = 1;

    public static boolean eligibleForAutoDriving = false;
}
