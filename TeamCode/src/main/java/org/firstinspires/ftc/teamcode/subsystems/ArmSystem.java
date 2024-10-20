package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;

import java.util.function.DoubleSupplier;


public class ArmSystem extends SubsystemBase {

    private DcMotorEx Pivot, Extension;

    private final Servo Wrist, Claw;


    private final PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetPosition, PivotTargetAngle, ExtensionTargetLength;


    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360;
    public DoubleSupplier CurrentExtensionLength = () -> ((Extension.getCurrentPosition() / 384.5) * 360 - CurrentPivotAngle.getAsDouble()) / 2088 * 696;


    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving
    private double backPedalStart = 0; // where pivot is when command to move starts

    private Vector2d targetClawPoint; // robot centric

    private boolean justPressedPreset = false;

    private double ArmFrameRate;

    ElapsedTime mRuntime;

    GamepadEx gamepad;

    Telemetry telemetry;


    public ArmSystem(HardwareMap map, GamepadEx operatorGamepad, Telemetry inputTelemetry) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        Extension = map.get(DcMotorEx.class, "Extension");
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
        gamepad = operatorGamepad;
        ClawTargetPosition = 0.5;
        WristTargetAngle = 180;

        mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = inputTelemetry;

        PivotPID = new PIDController(0.1, 0, 0, 0, Constants.pivotMaxAngle, 0,
                0.5, Constants.pivotMaxAngle, 2.5, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.012, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 150, 5, true, false,
                CurrentExtensionLength);
    }

    // Dictionary of things the arm has to do
    // 1. move claw to length and height position (robot centric)
    // 2. move claw to xyz position (field centric)
    // 3. pickup (robot centric)
    // 4. pickup (field centric)
    // 5. pickup from field wall
    // 6. score high basket
    // 7. score high rung
    // 8. hang 1st level
    // 9. hang 2nd level
    // 10.hang 3rd level (runs 2nd level hang first)

    // Low priority
    // 11.score low rung
    // 12.score middle basket
    // 13.score net zone


    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly

        ArmFrameRate = (1 / (mRuntime.time())) * 1000;
        mRuntime.reset();

        // Manual arm control when controllers active
        if (manualArmControlActive()) {
            backPedalExtension = false;
            Vector2d currentClawPos = getCurrentClawPoint();
            if (Math.abs(gamepad.getLeftY()) > 0.03 || Math.abs(gamepad.getRightY()) > 0.03) {
                double X = currentClawPos.x + Constants.maxManualClawSpeedHorizontal * gamepad.getLeftY() / ArmFrameRate;
                double Y = currentClawPos.y + Constants.maxManualClawSpeedVertical * gamepad.getRightY() / ArmFrameRate;
                moveArmToPoint(new Vector2d(X, Y));
            }
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && !justPressedPreset) { // presets
            resetArm();
            justPressedPreset = true;
        } else justPressedPreset = false;


        if (gamepad.getButton(GamepadKeys.Button.Y)) {
            WristTargetAngle = 190;
        } else if (gamepad.getButton(GamepadKeys.Button.X)) {
            WristTargetAngle = 90;
        } else if (gamepad.getButton(GamepadKeys.Button.A)) {
            WristTargetAngle = 0;
        }


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

        Extension.setPower(ExtensionPID.getPower());

        if ((CurrentExtensionLength.getAsDouble() < 80 || CurrentPivotAngle.getAsDouble() > 45) && WristTargetAngle < 80) WristTargetAngle = 90;

        setWrist(WristTargetAngle - PivotPID.encoderPosition.getAsDouble());

        Claw.setPosition(ClawTargetPosition - (0.05 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        telemetry.addData("Extension Target Length", ExtensionTargetLength);
        telemetry.addData("Extension Current Length", CurrentExtensionLength.getAsDouble());
        telemetry.addData("Pivot Target Angle", PivotTargetAngle);
        telemetry.addData("Pivot Current Angle", CurrentPivotAngle.getAsDouble());
        telemetry.addData("Is backpedaling Arm", backPedalExtension);
        if (manualArmControlActive()) telemetry.addLine("Arm Control is AUTO");
        else telemetry.addLine("Arm Control is Manual");
        telemetry.addData("Wrist Target Angle", WristTargetAngle);
        telemetry.addData("Claw Position", ClawTargetPosition);


    }


    public void setWrist(double Angle) { Wrist.setPosition(1.35 * (Angle / 360) + 0.31); } // make wrist go to that specific angle


    private boolean manualArmControlActive() {
        return (Math.abs(gamepad.getLeftY()) > 0.03 || Math.abs(gamepad.getRightY()) > 0.03);
    }


    private void moveArmToPoint(Vector2d point) { // units:mm, makes pivot and extension work together to go to a set point relative to the robot
        PivotTargetAngle = Math.toDegrees(Math.atan2(point.y, point.x)); // forward 0 is at the pivot point, HEIGHT 0 IS FROM AXLE not from floor
        ExtensionTargetLength = Math.hypot(point.x, point.y);
        if (PivotTargetAngle < 0) PivotTargetAngle = 0;
        else if (PivotTargetAngle > Constants.pivotMaxAngle) PivotTargetAngle = Constants.pivotMaxAngle;
        if (ExtensionTargetLength < 0) ExtensionTargetLength = 0;
        else if (ExtensionTargetLength > Constants.extensionMaxLength) ExtensionTargetLength = Constants.extensionMaxLength;
    }


    private Vector2d getCurrentClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((CurrentExtensionLength.getAsDouble() + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(CurrentPivotAngle.getAsDouble())),
                (CurrentExtensionLength.getAsDouble() + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(CurrentPivotAngle.getAsDouble())));
    }


    private Vector2d getTargetClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((ExtensionTargetLength + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(PivotTargetAngle)),
                (ExtensionTargetLength + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(PivotTargetAngle)));
    }


    public void moveClawToFieldCoordinate(Vector2d TargetClawPos, double TargetHeight) {
        // Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();

    }


    public Pose2d getCurrentClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();
        Vector2d CurrentClawPoint = getCurrentClawPoint(); // Claw Position relative to robot
        Vector2d CurrentClawPosition = (new Vector2d(
                CurrentClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                CurrentClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position);

        return new Pose2d((new Vector2d(
                CurrentClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                CurrentClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
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

    /*
    public void toggleClaw() {
        if (ClawTargetPosition < 70) ClawTargetPosition = 90;
        else ClawTargetPosition = 0;
    }
    */

    public void openClaw() { ClawTargetPosition = 0.5; }
    public void closeClaw() { ClawTargetPosition = 0.85; }

}
