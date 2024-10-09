package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class ArmSystem {

    private DcMotorEx Pivot, Extension;

    private Servo Wrist, Claw;


    private double ClawTargetX, ClawTargetY, ClawTargetZ;


    public ArmSystem(HardwareMap map) {
        Pivot = map.get(DcMotorEx.class, "Pivot");
        Extension = map.get(DcMotorEx.class, "Extension");
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
    }

    // Dictionary of Arm Modes
    // MoveMode:
    // 0 = move claw to xyz position (robot centric)
    // 1 = move claw to xyz position (field centric)
    // PickupMode:
    // 0 = pickup (robot centric)
    // 1 = pickup (field centric)
    // 2 = pickup from field wall
    // ScoreSampleMode:
    // 0 = score net zone (low priority)
    // 1 = score middle basket (low priority)
    // 2 = score high basket
    // ScoreSpecimenMode:
    // always score high rung
    // HangMode:
    // 0 = hang 1st level
    // 1 = hang 2nd level
    // 2 = hang 3rd level (runs 2nd level hang first)

    public void MoveClawArm(double MoveMode, double setClawTargetX, double setClawTargetY, double setClawTargetZ) {
        ClawTargetX = setClawTargetX;
        ClawTargetY = setClawTargetY;
        ClawTargetZ = setClawTargetZ;

    }


    public void PickupSample(double PickupMode) {

    }


    public void ScoreSample(double ScoreSampleMode) {

    }


    public void ScoreSpecimen() {

    }


    public void Hang(double HangMode) {

    }

}
