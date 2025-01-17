package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Settings {

    public static double maxDrivetrainMotorPower = 0.85;
    public static double maxDrivetrainTurnPower = 0.8;
    public static double minimumPivotSpeedPercent = 0.4; // for when extension is fully extended
    public static double pivotMaxAngle = 90; // degrees
    public static double maxManualClawSpeedVertical = 700;
    public static double maxManualClawSpeedHorizontal = 600; // mm per second
    public static double maxManualExtensionSpeed = 700;
    public static double maxManualPivotSpeed = 70;
    public static double maxCameraTargetingSpeed = 150;
    public static double maxCameraTargetingTurnSpeed = 0.5; // mm, percent
    public static double ClawOpenPosition = 0.5; // servo units
    public static double ClawClosedPosition = ClawOpenPosition + 0.35; // servo units
    public static double controllerDeadZone = 0.025;
    public static double pivotRetractedGravityPower = 0.156;
    public static double pivotExtendedGravityPower = 0.3;
    public static double extensionGravityPower = 0.166;
    public static double driveFeedBackStaticPower = 0.065; // power required in order to start moving the robot
    public static double pivotMotorBacklash = 13; // degrees
    public static double LinearSlideBend = -8; // degrees that the linear slide bends when horizontal and at max extension
    public static double ServoMSPerDegree = 6; // time is takes for the servo to move 1 degree
    public static double SwerveMinimumPower = 0.05;
    public static double WristServoRatio = 1.7;
    public static double WristServoOffset = 15;
    public static double WristFloorAngle = 210;
    public static int[] AcceptableIds = new int[] {1, 2, 3}; // 1 = red, 2 = yellow, 3 = blue
    public static double HuskyLensTargetX = 160;
    public static double HuskyLensTargetY = 110; // huskylens screen is 320 pixels width (x) and 240 pixels height (y)
    public static int SwerveModuleDriveSharpness = 1; // 1 is normal, a value higher than 1 would make the module have much less power when not at the correct angle
    public static double OperatorTurnOverridePower = 0.5;


    public static class ExtensionPIDVariables{
        public static double kP = 0.018;
        public static double kI = 0;
        public static double kD = 0.0005;
    }
    public static class PivotPIDVariables{
        public static double kP = 0.06;
        public static double kI = 0;
        public static double kD = 0;
        public static double maxSpeed = 180;
    }
    public static class HeadingPIDVariables{
        public static double kP = 0.008;
        public static double kI = 0;
        public static double kD = 0.0004;
        public static double minDifference = 0.25;
    }
}
