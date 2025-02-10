package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDControllerSettingsReference;

@Config
public class Settings {

    public static double maxDrivetrainMotorPower = 0.9; // percent
    public static double maxDrivetrainTurnPower = 0.7;
    public static double controllerDeadZone = 0.025;
    public static double driveFeedBackStaticPower = 0.065; // power required in order to start moving the robot
    public static double SwerveMinimumPower = 0.05;
    public static int[] AcceptableIds = new int[] {1, 2, 3}; // 1 = red, 2 = yellow, 3 = blue
    public static double HuskyLensTargetX = 160;
    public static double HuskyLensTargetY = 70; // huskylens screen is 320 pixels width (x) and 240 pixels height (y)
    public static int SwerveModuleDriveSharpness = 1; // 1 is normal, a value higher than 1 would make the module have much less power when not at the correct angle
    public static double DriveExtensionDriveReduction = 0.2;
    public static double DriveExtensionTurnReduction = 0;
    public static double DriverLocalizationCorrectionHoldTime = 1000; // ms
    public static double LocalizationChamberResetY = tiles(-1.3), LocalizationRightResetX = tiles(2.75), LocalizationBottomResetY = tiles(-3) + 4.0, LocalizationLeftResetX = tiles(-2.75);

    @Config
    public static class FrameRateStabilizerEditor {

        public static boolean FrameRateStabilizerEnabled = false;
        public static double FrameRateStabilizerRatio = 0.75; // percent
        public static double FrameRateStabilizerMaxFrames = 20;
        public static double FrameRateStabilizerMinFrames = 5;
        public static double FrameRateStabilizerMaxWait = 100; // milliseconds
    }

    @Config
    public static class ArmSystemSettings {

        public static double minimumPivotSpeedPercent = 0.4; // for when extension is fully extended
        public static double pivotMaxAngle = 100; // degrees
        public static double maxManualClawSpeedVertical = 700;
        public static double maxManualClawSpeedHorizontal = 600; // mm per second
        public static double maxManualExtensionSpeed = 750;
        public static double maxManualPivotSpeed = 70;
        public static double maxCameraTargetingSpeed = 250;
        public static double maxCameraTargetingSpeedSquared = 0;
        public static double maxCameraTargetingTurnSpeed = 0.5; // mm, percent
        public static double pivotRetractedGravityPower = 0.156;
        public static double pivotExtendedGravityPower = 0.3;
        public static double extensionGravityPower = 0.166;
        public static double pivotMotorBacklash = 13; // degrees
        public static double LinearSlideBend = -8; // degrees that the linear slide bends when horizontal and at max extension
        public static double WristServoMSPerDegree = 0; // time is takes for the servo to move 1 degree
        public static double WristServoRatio = 1.38; //1.7 axon // og 1.35
        public static double WristServoOffset = 0;
        public static double WristFloorAngle = 205;
        public static boolean WristServoReversed = true;
        public static double OperatorTurnOverridePower = 0; // permanently 0
        public static double ClawClosingSlowPower = -0.2;
        public static double ClawOpeningPower = 1; // degrees (0.5)
        public static double ClawClosingPower = -1; // degrees (0.85)
        public static double ChamberPresetPivotAngle = 67.5;
        public static double ChamberPresetExtensionLength = 200;
        public static double PivotBacklashMaxAngle = 82;
        public static double HumanPlayerPresetPivotAngle = 72;
        public static double OperatorDriveTurnPower = 0.35;
        public static double ExtensionNormalMaxAcceleration = 0;
        public static double ExtensionNormalMaxDeceleration = -696;
        public static double ExtensionSubMaxAcceleration = 0;
        public static double ExtensionSubMaxDeceleration = -696;
        public static double EmergencyExtensionPowerReleaseTimeout = 30000; // ms
        public static double EmergencyExtensionPowerReleaseSpeed = 0.1; // power that is decreased every second after timeout
        public static double PivotAccelerationDampeningPower = 0;

        public static PIDControllerSettingsReference ExtensionReference = new PIDControllerSettingsReference(
                0.018,
                0,
                0.0005,
                1,
                0,
                Constants.extensionMaxLength,
                0,
                1,
                0,
                0,
                696,
                15,
                ExtensionNormalMaxAcceleration,
                ExtensionNormalMaxDeceleration,
                true,
                true);

        public static PIDControllerSettingsReference PivotReference = new PIDControllerSettingsReference(
                0.06,
                0,
                0,
                1,
                0,
                ArmSystemSettings.pivotMaxAngle,
                0,
                1,
                0,
                0,
                180,
                3,
                0,
                -200,
                true,
                true);
    }

    public static double slowMovingHeadingPIDMinDifference = 3;

    public static PIDControllerSettingsReference HeadingReference = new PIDControllerSettingsReference(
            0.007,
            0,
            0.0004,
            1,
            0,
            0,
            0,
            1,
            0,
            0.25,
            0,
            0,
            0,
            0,
            false,
            false);


    @Config
    public static class DriverAutoTargetCoords{
        public static double BasketX = -2.45, BasketY = -2.4;

        public static double HumanPickupX = 2.1, HumanPickupY = -2.5;
        public static double HumanDropOffX = 2.1, HumanDropOffY = -2.2, HumanDropOffHeading = 315;

        public static double SubmersibleX = -1.05, SubmersibleY = 0;

    }

}
