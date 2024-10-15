package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

import java.util.function.DoubleSupplier;


public class ArmSystem extends SubsystemBase {

    private DcMotorEx Pivot, Extension;

    private Servo Wrist, Claw;


    private PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetAngle, PivotTargetAngle, ExtensionTargetLength;


    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360;
    public DoubleSupplier CurrentExtensionLength = () -> ((Extension.getCurrentPosition() / 384.5) * 360) / 2088 * 696;


    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving
    private double backPedalStart = 0; // where pivot is when command to move starts


    public ArmSystem(HardwareMap map, GamepadEx gamepad) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        Extension = map.get(DcMotorEx.class, "Extension");
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");

        PivotPID = new PIDController(0.1, 0, 0, 0, Constants.pivotMaxAngle, 0,
                0.5, Constants.pivotMaxAngle, 2.5, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.012, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 150, 5, true, false,
                CurrentExtensionLength);
    }

    // Dictionary of Arm Modes
    // 0 : move claw to length and height position (robot centric)
    // 1 : move claw to xyz position (field centric)
    // 2 : pickup (robot centric)
    // 3 : pickup (field centric)
    // 4 : pickup from field wall
    // 5 : score net zone (low priority)
    // 6 : score middle basket (low priority)
    // 7 : score high basket
    // 8 : always score high rung
    // 9 : hang 1st level
    // 10 : hang 2nd level
    // 11 : hang 3rd level (runs 2nd level hang first)


    public void setWrist(double Angle) { WristTargetAngle = Angle; }
    public void setClaw(double Angle) { ClawTargetAngle = Angle; }


    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly

        // slows pivot down when the extension is extended
        PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (ExtensionPID.encoderPosition.getAsDouble() / (ExtensionPID.maxPosition - ExtensionPID.minPosition)));

        PivotPID.setTarget(PivotTargetAngle);
        Pivot.setPower(PivotPID.getPower());

        // if backpedal is enabled and the angle already traveled is less than half of the total angle that needs to be traversed
        if (backPedalExtension && (PivotPID.encoderPosition.getAsDouble() - backPedalStart) < (PivotTargetAngle - backPedalStart) / 2) {
            ExtensionPID.setTarget(0); // move extension towards 0
        } else {
            ExtensionPID.setTarget(ExtensionTargetLength);
            backPedalExtension = false;
        }

        // TODO: make it account for the pivot angle to prevent skipping
        Extension.setPower(ExtensionPID.getPower());

        Wrist.setPosition((WristTargetAngle - PivotPID.encoderPosition.getAsDouble() + Constants.wristOffset) / 360);

        Claw.setPosition((ClawTargetAngle + Constants.clawOffset) / 360);
    }


    private void moveArmToPoint(double forward, double height) { // units:mm, makes pivot and extension work together to go to a set point relative to the robot
        PivotTargetAngle = Math.toDegrees(Math.atan2(height, forward)); // forward 0 is at the pivot point, HEIGHT 0 IS FROM AXLE not from floor
        ExtensionTargetLength = Math.hypot(forward, height);
        if (PivotTargetAngle < 0) PivotTargetAngle = 0;
        else if (PivotTargetAngle > Constants.pivotMaxAngle) PivotTargetAngle = Constants.pivotMaxAngle;
        if (ExtensionTargetLength < 0) ExtensionTargetLength = 0;
        else if (ExtensionTargetLength > Constants.extensionMaxLength) ExtensionTargetLength = Constants.extensionMaxLength;
    }


    public void moveClawToFieldCoordinate(double X, double Y) {

    }


    public void moveClawToTopBasket() {
        if (Math.abs(90 - PivotPID.encoderPosition.getAsDouble()) > 15) backPedalExtension = true; // only backpedal if the pivot has to rotate more than set degrees
        backPedalStart = PivotPID.encoderPosition.getAsDouble();
        PivotTargetAngle = 90;
        ExtensionTargetLength = 696;
        WristTargetAngle = 200;
    }


    public void moveClawIntoSubmersible() {
        // nothing yet
    }


    public void resetArm() {
        PivotTargetAngle = 0;
        ExtensionTargetLength = 0;
        WristTargetAngle = 180;
    }


    public void toggleClaw() {
        if (ClawTargetAngle < 70) ClawTargetAngle = 90;
        else ClawTargetAngle = 0;
    }

}
