package org.firstinspires.ftc.teamcode;

public class Constants {

    public static double encoderResolution = 8192; // differential swerve module encoders

    public static double wristOffset = 90; // degrees
    public static double clawOffset = 0; // servo units
    public static double minimumPivotSpeedPercent = 0.3; // for when extension is fully extended

    public static double extensionMaxLength = 696;
    public static double pivotMaxAngle = 90;
    public static double pivotAxleHeight = 82.242420; //mm not including how much the robot sinks into the tiles
    public static double pivotAxleOffset = -156.0; //mm toward the back of the robot from center of rotation
    public static double retractedExtensionLength = 264; //mm from center of pivot axle to end of linear slide when retracted

    public static double maxManualClawSpeedVertical = 100; // mm per second
    public static double maxManualClawSpeedHorizontal = 100; // mm per second

}
