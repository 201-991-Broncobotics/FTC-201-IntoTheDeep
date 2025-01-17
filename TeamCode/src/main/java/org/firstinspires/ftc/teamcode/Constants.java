package org.firstinspires.ftc.teamcode;


public class Constants {

    public static double ModuleGearRatio = (21 / 58.0) * (35 / 9.0) * (63.5 * Math.PI / 25.4); // units are in inches (move along the ground) per 1 motor revolution
    private static final int SlidePulleyTeeth = 72; // The gobilda one has 60 teeth though it was slow and the belt skipped too often
    public final static double SpoolDegreesToMaxExtension = (696 * 360) / (0.02506377057 * 25.4 * Math.PI * SlidePulleyTeeth); // default is 2088 with the 60 tooth pulley
    public static double encoderResolution = 8192; // Rev encoder resolution
    public static double tileLength = 23.5625; // inches
    public static double extensionMaxLength = 696; // mm
    public static double pivotAxleHeight = 82.242420; //mm not including how much the robot sinks into the tiles
    public static double pivotAxleOffset = -156.0; //mm of distance toward the back of the robot from center of robot
    public static double retractedExtensionLength = 264; //mm from center of pivot axle to end of linear slide when retracted
    public static double wristLength = 185; //mm from the front of slides to where the claw can best pickup a sample off the ground when wrist is straight
    public static double horizontalExpansionLimit = 1066.8; // in millimeters

    // 420 mm from end of claw to pivot axle, 457.2 is the length of the robot, 15 is extra distance to be sure
    public static double freeHorizontalExpansion = horizontalExpansionLimit - 457.2 - 15; // distance that is left within the horizontal expansion limit for the linear slide
    public static double TrackWidth = 11.188976; // inches
    public static double PivotDownAngle = 5.5;

}
