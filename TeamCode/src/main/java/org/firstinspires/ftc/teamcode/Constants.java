package org.firstinspires.ftc.teamcode;

public class Constants {

    public static double encoderResolution = 8192; // differential swerve module encoders

    public static double minimumPivotSpeedPercent = 0.4; // for when extension is fully extended
    public static double extensionMaxLength = 696;
    public static double pivotMaxAngle = 90;
    public static double pivotAxleHeight = 82.242420; //mm not including how much the robot sinks into the tiles
    public static double pivotAxleOffset = -156.0; //mm of distance toward the back of the robot from center of robot
    public static double retractedExtensionLength = 264; //mm from center of pivot axle to end of linear slide when retracted

    public static double maxManualClawSpeedVertical = 200, maxManualClawSpeedHorizontal = 200; // mm per second
    public static double maxManualExtensionSpeed = 300, maxManualPivotSpeed = 30; // mm, degrees

    public static double ClawOpenPosition = 0.5;
    public static double ClawClosedPosition = ClawOpenPosition + 0.35;

    public static double controllerDeadZone = 0.03; // Not really necessary but anyways
    public static double pivotRetractedGravityPower = 0.156, pivotExtendedGravityPower = 0.497, extensionGravityPower = 0.080; // 0.097

    public static double horizontalExpansionLimit = 1066.8; // in millimeters
    // 420 mm from end of claw to pivot axle, 457.2 is the length of the robot, 15 is extra distance to be sure
    public static double freeHorizontalExpansion = horizontalExpansionLimit - 457.2 - 15; // distance that is left within the horizontal expansion limit
}
