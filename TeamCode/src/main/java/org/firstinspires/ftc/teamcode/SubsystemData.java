package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

public class SubsystemData {

    // This needs to be here as each subsystem can't interact or communicate with each other directly because command based.

    public static Pose2d CurrentRobotPose; // from roadrunner

    public static Pose2d CurrentPedroPose; // from Pedro Pathing

    public static double FrameRate = 20; // not actual value, it is constantly updated in ArmSystem


    public static boolean inTeleOp; // to stop auton in command based

    public static boolean absoluteDriving;

    public static boolean OverrideDrivetrainRotation = false; // for auto aiming
    public static double OverrideDrivetrainTargetHeading = 0;


    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PIDs

    public static Pose2d AutonError = new Pose2d(new Vector2d(0, 0), 0);

    public static boolean IMUWorking;
    public static IMU imuInstance;

    public static double IMUZero = 0; // in radians

    public static boolean needToResetIMU = false;

    // public static double[] DriveMotorHighCurrents = new double[] {0, 0, 0, 0};

    public static HuskyLens.Block[] Vision;

    public static double CameraTargetPixelsX, CameraTargetPixelsY, CameraTargetsPixelsWidth;

    public static int[] AcceptableIds = new int[] {1, 2, 3}; // 1 = red, 2 = yellow, 3 = blue

    public static double HuskyLensTargetX = 160, HuskyLensTargetY = 110; // huskylens screen is 320 pixels width (x) and 240 pixels height (y)
    // note that 0,0 on the huskylens is the top left corner

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

    public static int SwerveModuleDriveSharpness = 1; // 1 is normal, a value higher than 1 would make the module have much less power when not at the correct angle

    public static boolean HoldClawFieldPos = false;


    public static double OperatorTurningPower = 0;
    public static double AutoAimHeading = 0;

    public static double HeadHoldTarget = 0;

    public static boolean NeedToRealignHeadingHold = false;


    public static boolean repeatForwardBack = false;




    public static double AutonStoppingDistance = 0.5; // angle difference is actually in radians unintentionally


    public static double targetPosePerpOffset = 0; // confusing to explain but it supposed to prevent the robot driving in circles around the target position

    public static double VelocityTargetDirectionDifference = 0;
    public static double VelocityTargetDirectionDifferenceMax = 55;
    public static double VelocityTargetDirectionDifferenceMaxVelocity = 5; // extra long variable names



    public static boolean AutoAimingForWall = false; // false means auto aim for ground pickup
    public static double AutoAimForwardGain = 0;
    public static double AutoAimStrafeGain = 0;

    public static double HighDriveVel = 0, HighAngVel = 0, HighDriveAccel = 0, LowDriveAccel = 0, HighAngAccel = 0;

    public static boolean alreadyAlignedArm = false;

    public static PoseVelocity2d RobotVelocity; // constantly updates with localizer instead of requiring updating localizer an extra time

}
