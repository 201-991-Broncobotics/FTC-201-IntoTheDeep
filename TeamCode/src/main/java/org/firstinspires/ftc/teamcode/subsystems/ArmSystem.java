package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

import java.util.function.DoubleSupplier;


public class ArmSystem {

    private DcMotorEx Pivot, Extension;

    private Servo Wrist, Claw;


    private PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetAngle, PivotTargetAngle, ExtensionTargetLength;


    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360;
    public DoubleSupplier CurrentExtensionLength = () -> ((Extension.getCurrentPosition() / 384.5) * 360) / 2088 * 696;


    public boolean ArmInManualControl = false;


    public ArmSystem(HardwareMap map) {
        Pivot = map.get(DcMotorEx.class, "Pivot");
        Extension = map.get(DcMotorEx.class, "Extension");
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");

        PivotPID = new PIDController(0.1, 0, 0, 0, 90, 0,
                0.5, 90, 2.5, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.012, 0, 0, 0, 696, 0,
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


    public void MoveArmToTargets() { // VERY IMPORTANT that this needs to be looping constantly

        // prevents pivot from going max speed while fully extended
        PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (ExtensionPID.encoderPosition.getAsDouble() / (ExtensionPID.maxPosition - ExtensionPID.minPosition)));

        PivotPID.setTarget(PivotTargetAngle);
        Pivot.setPower(PivotPID.getPower());
        ExtensionPID.setTarget(ExtensionTargetLength);
        Extension.setPower(ExtensionPID.getPower());

        Wrist.setPosition((WristTargetAngle + Constants.wristOffset) / 360);

        Claw.setPosition((ClawTargetAngle + Constants.clawOffset) / 360);
    }

}
