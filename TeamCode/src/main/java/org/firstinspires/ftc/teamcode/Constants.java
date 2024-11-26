package org.firstinspires.ftc.teamcode;

public class Constants {

    public static double maxDrivetrainMotorPower = 0.9; // TODO: Make sure this is what it needs to be before competition

    public static double ModuleGearRatio = (21 / 58.0) * (35 / 9.0) * (63.5 * Math.PI / 25.4); // units are in inches (move along the ground) per 1 motor revolution
    private static final int SlidePulleyTeeth = 72; // The gobilda one has 60 teeth though it was slow and the belt skipped too often
    public final static double SpoolDegreesToMaxExtension = (696 * 360) / (0.02506377057 * 25.4 * Math.PI * SlidePulleyTeeth); // default is 2088 with the 60 tooth pulley

    public static double encoderResolution = 8192; // differential swerve module encoder resolution

    public static double tileLength = 23.5625; // inches

    public static double minimumPivotSpeedPercent = 0.4; // for when extension is fully extended
    public static double extensionMaxLength = 696; // mm
    public static double pivotMaxAngle = 90; // degrees
    public static double pivotAxleHeight = 82.242420; //mm not including how much the robot sinks into the tiles
    public static double pivotAxleOffset = -156.0; //mm of distance toward the back of the robot from center of robot
    public static double retractedExtensionLength = 264; //mm from center of pivot axle to end of linear slide when retracted

    public static double maxManualClawSpeedVertical = 700, maxManualClawSpeedHorizontal = 600; // mm per second
    public static double maxManualExtensionSpeed = 700, maxManualPivotSpeed = 80, maxManualHeadingSpeed = 30; // mm, degrees, degrees per second

    public static double maxCameraTargetingSpeed = 500, maxCameraTargetingTurnSpeed = 50; // mm, degrees

    public static double ClawOpenPosition = 0.5; // servo units
    public static double ClawClosedPosition = ClawOpenPosition + 0.35; // servo units

    public static double ClawMiddlePosition = ClawOpenPosition + 0.1; // servo units

    public static double controllerDeadZone = 0.025;
    public static double pivotRetractedGravityPower = 0.156, pivotExtendedGravityPower = 0.497, extensionGravityPower = 0.080;

    public static double horizontalExpansionLimit = 1066.8; // in millimeters

    // 420 mm from end of claw to pivot axle, 457.2 is the length of the robot, 15 is extra distance to be sure
    public static double freeHorizontalExpansion = horizontalExpansionLimit - 457.2 - 15 + 50; // distance that is left within the horizontal expansion limit

    public static double TrackWidth = 11.188976; // inches


}
