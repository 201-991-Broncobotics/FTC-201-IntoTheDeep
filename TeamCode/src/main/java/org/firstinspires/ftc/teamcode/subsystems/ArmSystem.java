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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemDataTransfer;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.function.DoubleSupplier;


public class ArmSystem extends SubsystemBase {

    private DcMotorEx Pivot, Extension;
    private final Servo Wrist, Claw;
    private final PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetPosition, PivotTargetAngle, ExtensionTargetLength;

    private double CurrentPivotAngleZero = 0, CurrentExtensionLengthZero = 0;
    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360 - CurrentPivotAngleZero;
    public DoubleSupplier CurrentExtensionLength = () -> ((Extension.getCurrentPosition() / 384.5) * 360 + CurrentPivotAngle.getAsDouble()) / 2088 * 696 - CurrentExtensionLengthZero;

    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving
    private double backPedalStart = 0; // where pivot is when command to move starts

    private boolean justPressedPreset = false;

    private boolean resettingExtensionZero = false;

    ElapsedTime ArmFrameTime;
    GamepadEx gamepad;
    Telemetry telemetry;

    // PID tuning stuff
    GamepadEx driverGamepad;
    ElapsedTime PIDButtonPressTime;
    boolean PIDButtonPressed = false;
    boolean PIDIncrementButtonPressed = false;
    double PIDVar = 0;
    double PIDChangeIncrement = 0.01;

    HuskyLensCamera huskyLens;

    double LastTestingTime = 0;


    public ArmSystem(HardwareMap map, GamepadEx operatorGamepad, Telemetry inputTelemetry, GamepadEx driver) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        Extension = map.get(DcMotorEx.class, "Extension");
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
        gamepad = operatorGamepad;
        ClawTargetPosition = Constants.ClawOpenPosition; // open claw
        WristTargetAngle = 180; // point claw straight up

        ArmFrameTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        PIDButtonPressTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        driverGamepad = driver;
        telemetry = inputTelemetry;

        PivotPID = new PIDController(0.1, 0, 0, 0, Constants.pivotMaxAngle, 0,
                1, Constants.pivotMaxAngle, 2.5, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.012, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 0, 5, true, false,
                CurrentExtensionLength);

        huskyLens = new HuskyLensCamera(map);
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
        double armFrameRate = 1 / (ArmFrameTime.time() / 1000.0); // in seconds
        ArmFrameTime.reset(); // timer is in milliseconds
        telemetry.addData("Arm System FrameRate:", armFrameRate);
        telemetry.addData("Drive System FrameRate:", SubsystemDataTransfer.driveSystemFrameRate);
        telemetry.addLine(" ");

        LastTestingTime = 0;

        // Manual arm control when controllers active
        if (manualArmControlActive()) {
            backPedalExtension = false;
            Vector2d targetClawPos = getTargetClawPoint();
            if (armFrameRate > 2) {
                moveArmToPoint(new Vector2d(
                        targetClawPos.x + Constants.maxManualClawSpeedHorizontal * gamepad.getLeftY() / armFrameRate,
                        targetClawPos.y + Constants.maxManualClawSpeedVertical * gamepad.getRightY() / armFrameRate
                ));
            }
            justPressedPreset = false; // also reset button presses
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && !justPressedPreset) { // Presets
            resetArm();
            justPressedPreset = true;
        } else justPressedPreset = false;

        // Wrist and Claw
        if (gamepad.getButton(GamepadKeys.Button.Y)) {
            WristTargetAngle = 190;
        } else if (gamepad.getButton(GamepadKeys.Button.X)) {
            WristTargetAngle = 90;
        } else if (gamepad.getButton(GamepadKeys.Button.A)) {
            WristTargetAngle = 0;
        }

        telemetry.addData("Point 1:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();


        // PIVOT

        // slows down pivot based on how far the extension is extended
        PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLength.getAsDouble() / Constants.extensionMaxLength));

        PivotPID.setTarget(PivotTargetAngle);
        // set pivot power to pid value + the amount of power needed to counteract gravity at the current pivot angle and current extension length
        double PivotPIDPower = PivotPID.getPower();
        double PivotPower = PivotPIDPower + ((Constants.pivotExtendedGravityPower - Constants.pivotRetractedGravityPower)
                / Constants.extensionMaxLength * CurrentExtensionLength.getAsDouble() + Constants.pivotRetractedGravityPower)
                * Math.cos(Math.toRadians(CurrentPivotAngle.getAsDouble()));
        if (Math.abs(PivotPower) > 0.75) PivotPower = Math.signum(PivotPower) * 0.75; // sets max pivot power TODO: remove this after it has been tested that it works
        if ((CurrentPivotAngle.getAsDouble() < 3 && PivotTargetAngle < 3) || armFrameRate < 2) { // stop pivot if it is resting on the mechanical stop or if the framerate is less than 2
            Pivot.setPower(0);
        } else Pivot.setPower(PivotPower);

        telemetry.addData("Point 2:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();


        // EXTENSION

        // The purpose of Backpedal is to retract the extension when the pivot needs to move a lot so that the arm is less likely to hit something and the pivot can rotate faster
        // if backpedal is enabled and the angle already traveled is less than half of the total angle that needs to be traversed
        if (backPedalExtension && (CurrentPivotAngle.getAsDouble() - backPedalStart) < (PivotTargetAngle - backPedalStart) / 2) {
            ExtensionPID.setTarget(0); // move extension towards 0
        } else {
            ExtensionPID.setTarget(ExtensionTargetLength);
            backPedalExtension = false;
        }

        double ExtensionPIDPower = ExtensionPID.getPower();
        double ExtensionPower = ExtensionPIDPower + Math.sin(Math.toRadians(CurrentPivotAngle.getAsDouble())) * Constants.extensionGravityPower;
        if (armFrameRate < 2) ExtensionPower = 0; // emergency stop extension if framerate is less than 2
        Extension.setPower(ExtensionPower);

        if (resettingExtensionZero && ExtensionTargetLength == 0) {
            // if Extension is close to where it thinks 0 is, tell it to keep retracting
            // if (CurrentExtensionLength.getAsDouble() < 10) Extension.setPower(-0.2);
            // resets 0 if motor can't move and resetting extension zero is enabled
            if (Extension.getCurrent(CurrentUnit.AMPS) > 8) { // TODO: make sure this isn't a problem when the arm gets temporarily stuck extended
                CurrentExtensionLengthZero = CurrentExtensionLength.getAsDouble();
                resettingExtensionZero = false;
            }
        } else resettingExtensionZero = false;
        // If the current extension length is ever negative, set the current value to zero
        if (CurrentExtensionLength.getAsDouble() < 0) CurrentExtensionLengthZero = CurrentExtensionLength.getAsDouble();

        telemetry.addData("Point 3:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();

        // WRIST AND CLAW

        if ((CurrentExtensionLength.getAsDouble() < 80 || CurrentPivotAngle.getAsDouble() > 45) && WristTargetAngle < 80) {
            WristTargetAngle = 90;
        }
        setWrist(WristTargetAngle - CurrentPivotAngle.getAsDouble());

        // right trigger slightly loosens the claw's grip
        Claw.setPosition(ClawTargetPosition - (0.05 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        telemetry.addData("Point 4:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();

        if (!SubsystemDataTransfer.IMUWorking) telemetry.addLine("IMU HATH BEEN DEFEATED");
        telemetry.addData("Heading:", Math.toDegrees(SubsystemDataTransfer.getCurrentRobotPose().heading.toDouble()));
        telemetry.addLine("Robot Pose (in) X: " +
                functions.round(SubsystemDataTransfer.getCurrentRobotPose().position.x, 2) + " Y: " +
                functions.round(SubsystemDataTransfer.getCurrentRobotPose().position.y, 2));
        telemetry.addLine(" ");
        telemetry.addLine("Claw Pose X: " +
                functions.round(getCurrentClawPose().position.x, 2) + " Y: " +
                functions.round(getCurrentClawPose().position.y, 2) + " H: " +
                functions.round(getCurrentClawHeight(), 2));
        telemetry.addLine("Target Claw Pose X: " +
                functions.round(getTargetClawPose().position.x, 2) + " Y: " +
                functions.round(getTargetClawPose().position.y, 2) + " H: " +
                functions.round(getTargetClawHeight(), 2));
        telemetry.addData("Extension Target Length:", ExtensionTargetLength);
        telemetry.addData("Extension Current Length:", CurrentExtensionLength.getAsDouble());
        telemetry.addData("Pivot Target Angle:", PivotTargetAngle);
        telemetry.addData("Pivot Current Angle:", CurrentPivotAngle.getAsDouble());
        telemetry.addData("Is backpedaling Arm:", backPedalExtension);
        if (manualArmControlActive()) telemetry.addLine("Arm Control in AUTO");
        else telemetry.addLine("Arm Control in Manual");
        telemetry.addData("Wrist Target Angle:", WristTargetAngle);
        telemetry.addData("Claw Position:", ClawTargetPosition);
        telemetry.addLine(" ");
        telemetry.addData("LT High Current", SubsystemDataTransfer.DriveMotorHighCurrents[0]);
        telemetry.addData("LB High Current", SubsystemDataTransfer.DriveMotorHighCurrents[1]);
        telemetry.addData("RT High Current", SubsystemDataTransfer.DriveMotorHighCurrents[3]);
        telemetry.addData("RB High Current", SubsystemDataTransfer.DriveMotorHighCurrents[2]);
        telemetry.addLine(" ");
        telemetry.addData("Pivot PID Power:", PivotPIDPower);
        telemetry.addData("Pivot Motor Power:", PivotPower);
        telemetry.addData("Extension PID Power:", ExtensionPIDPower);
        telemetry.addData("Extension Motor Power:", ExtensionPower);
        telemetry.addData("Extension Motor Current:", Extension.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Point 5:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();

        tunePIDsWithController(driverGamepad);

        telemetry.addData("Point 6:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();

        telemetry.addLine(" ");
        huskyLens.ScanForSample(telemetry); // TODO: move this to its own command

        telemetry.addData("Point 7:", ArmFrameTime.time() - LastTestingTime);
        LastTestingTime = ArmFrameTime.time();

    }


    private void tunePIDsWithController(GamepadEx inputGamepad) {
        // Allows changing PID variables whilst on the field during testing:
        if (inputGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) PIDChangeIncrement = 0.01;
        else PIDChangeIncrement = 0.0001;

        if (inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !PIDButtonPressed) { // cycle through which PID variable is going to be edited
            PIDVar = PIDVar + 1;
            if (PIDVar > 12) PIDVar = 0;
            PIDButtonPressed = true;
        } else if (inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && !PIDButtonPressed) {
            PIDVar = PIDVar - 1;
            if (PIDVar < 0) PIDVar = 12;
            PIDButtonPressed = true;
        } else if (!inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) PIDButtonPressed = false;

        // if the button is held for more than a second, add or subtract constantly
        if ((inputGamepad.getButton(GamepadKeys.Button.DPAD_UP) || inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) && !(PIDIncrementButtonPressed && ArmFrameTime.time() < 1000)) {
            if (inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) PIDChangeIncrement = -PIDChangeIncrement; // subtract if down

            if (PIDVar == 1) ExtensionPID.kP = functions.round(ExtensionPID.kP + PIDChangeIncrement, 4);
            else if (PIDVar == 2) ExtensionPID.kI = functions.round(ExtensionPID.kI + PIDChangeIncrement, 4);
            else if (PIDVar == 3) ExtensionPID.kD = functions.round(ExtensionPID.kD + PIDChangeIncrement, 4);
            else if (PIDVar == 4) Constants.extensionGravityPower = functions.round(Constants.extensionGravityPower + PIDChangeIncrement * 10, 3);
            else if (PIDVar == 5) PivotPID.kP = functions.round(PivotPID.kP + PIDChangeIncrement, 4);
            else if (PIDVar == 6) PivotPID.kI = functions.round(PivotPID.kI + PIDChangeIncrement, 4);
            else if (PIDVar == 7) PivotPID.kD = functions.round(PivotPID.kD + PIDChangeIncrement, 4);
            else if (PIDVar == 8) Constants.pivotRetractedGravityPower = functions.round(Constants.pivotRetractedGravityPower + PIDChangeIncrement * 10, 3);
            else if (PIDVar == 9) Constants.pivotExtendedGravityPower = functions.round(Constants.pivotExtendedGravityPower + PIDChangeIncrement * 10, 3);
            else if (PIDVar == 10) SubsystemDataTransfer.HeadingTargetPID.kP = functions.round(SubsystemDataTransfer.HeadingTargetPID.kP + PIDChangeIncrement, 4);
            else if (PIDVar == 11) SubsystemDataTransfer.HeadingTargetPID.kI = functions.round(SubsystemDataTransfer.HeadingTargetPID.kI + PIDChangeIncrement, 4);
            else if (PIDVar == 12) SubsystemDataTransfer.HeadingTargetPID.kD = functions.round(SubsystemDataTransfer.HeadingTargetPID.kD + PIDChangeIncrement, 4);
            // else if (PIDVar == 13) Constants.ClawOpenPosition = functions.round(Constants.ClawOpenPosition + PIDChangeIncrement * 10, 3);
            if (!PIDIncrementButtonPressed) { // only happens once when the button is first pressed
                PIDButtonPressTime.reset(); // set the time that the button started being pressed to 0
                PIDIncrementButtonPressed = true;
            }
        } else if (!inputGamepad.getButton(GamepadKeys.Button.DPAD_UP) && !inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            PIDIncrementButtonPressed = false;
        }
        if (PIDVar == 0) telemetry.addLine("Not Editing PIDs");
        else if (PIDVar == 1) telemetry.addData("Editing: Extension Kp - ", ExtensionPID.kP);
        else if (PIDVar == 2) telemetry.addData("Editing: Extension Ki - ", ExtensionPID.kI);
        else if (PIDVar == 3) telemetry.addData("Editing: Extension Kd - ", ExtensionPID.kD);
        else if (PIDVar == 4) telemetry.addData("Editing: Extension Gravity - ", Constants.extensionGravityPower);
        else if (PIDVar == 5) telemetry.addData("Editing: Pivot Kp - ", PivotPID.kP);
        else if (PIDVar == 6) telemetry.addData("Editing: Pivot Ki - ", PivotPID.kI);
        else if (PIDVar == 7) telemetry.addData("Editing: Pivot Kd - ", PivotPID.kD);
        else if (PIDVar == 8) telemetry.addData("Editing: Pivot Retracted Gravity - ", Constants.pivotRetractedGravityPower);
        else if (PIDVar == 9) telemetry.addData("Editing: Pivot Extended Gravity - ", Constants.pivotExtendedGravityPower);
        else if (PIDVar == 10) telemetry.addData("Editing: Heading Kp - ", SubsystemDataTransfer.HeadingTargetPID.kP);
        else if (PIDVar == 11) telemetry.addData("Editing: Heading Ki - ", SubsystemDataTransfer.HeadingTargetPID.kI);
        else if (PIDVar == 12) telemetry.addData("Editing: Heading Kd - ", SubsystemDataTransfer.HeadingTargetPID.kD);
        // else if (PIDVar == 13) telemetry.addData("Editing: Claw Offset - ", Constants.ClawOpenPosition);
    }


    private boolean manualArmControlActive() {
        return (functions.inUse(gamepad.getLeftY()) || functions.inUse(gamepad.getRightY()));
    }


    private Vector2d getCurrentClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((CurrentExtensionLength.getAsDouble() + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(CurrentPivotAngle.getAsDouble())),
                (CurrentExtensionLength.getAsDouble() + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(CurrentPivotAngle.getAsDouble())));
    }


    private Vector2d getTargetClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((ExtensionTargetLength + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(PivotTargetAngle)),
                (ExtensionTargetLength + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(PivotTargetAngle)));
    }


    public Pose2d getCurrentClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();
        Vector2d CurrentClawPoint = getCurrentClawPoint(); // Claw Position relative to robot
        return new Pose2d((new Vector2d(
                CurrentClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                CurrentClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public Pose2d getTargetClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();
        Vector2d TargetClawPoint = getTargetClawPoint(); // Claw Target Position relative to robot
        return new Pose2d((new Vector2d(
                TargetClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                TargetClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public double getCurrentClawHeight() { // FROM FIELD TILES instead of from pivot axle
        // this has to be separate from pose since I don't want to create data types or use lists
        return Math.sin(Math.toRadians(CurrentPivotAngle.getAsDouble())) *
                (CurrentExtensionLength.getAsDouble() + Constants.retractedExtensionLength) + Constants.pivotAxleHeight;
    }


    public double getTargetClawHeight() { // FROM FIELD TILES instead of from pivot axle
        // this has to be separate from pose since I don't want to create data types or use lists
        return Math.sin(Math.toRadians(PivotTargetAngle)) *
                (ExtensionTargetLength + Constants.retractedExtensionLength) + Constants.pivotAxleHeight;
    }




    // MOVE ARM METHODS AND PRESETS:
    private void moveArmToPoint(Vector2d point) { // units:mm, makes pivot and extension work together to go to a set point relative to the robot
        PivotTargetAngle = Math.toDegrees(Math.atan2(point.y, point.x)); // forward 0 is at the pivot point, HEIGHT 0 IS FROM AXLE not from floor
        ExtensionTargetLength = Math.hypot(point.x, point.y) - Constants.retractedExtensionLength;
        if (PivotTargetAngle < 0) PivotTargetAngle = 0;
        else if (PivotTargetAngle > Constants.pivotMaxAngle) PivotTargetAngle = Constants.pivotMaxAngle;
        if (ExtensionTargetLength < 0) ExtensionTargetLength = 0;
        else if (ExtensionTargetLength > Constants.extensionMaxLength) ExtensionTargetLength = Constants.extensionMaxLength;
    }


    public void moveClawToFieldCoordinate(Vector2d TargetClawPos, double TargetHeight) { // needs to be called constantly while in use, HEIGHT IS FROM FIELD TILES
        Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();
        Vector2d DeltaPose = TargetClawPos.minus(CurrentPose.position); // Vector of where the arm needs to go relative to the robot

        // tell the drivetrain to point towards the target point
        SubsystemDataTransfer.OverrideDrivetrainTargetHeading = Math.toDegrees(Math.atan2(DeltaPose.y, DeltaPose.x));
        SubsystemDataTransfer.OverrideDrivetrainRotation = true;

        moveArmToPoint(new Vector2d(Math.hypot(DeltaPose.x, DeltaPose.y) - Constants.pivotAxleOffset,
                TargetHeight - Constants.pivotAxleHeight));
    }


    public void moveClawToTopBasket() {
        if (Math.abs(90 - CurrentPivotAngle.getAsDouble()) > 15) backPedalExtension = true; // only backpedal if the pivot has to rotate more than set degrees
        backPedalStart = CurrentPivotAngle.getAsDouble();
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
        resettingExtensionZero = true;
    }



    // WRIST AND CLAW METHODS

    public void setWrist(double Angle) { Wrist.setPosition(1.35 * (Angle / 360) + 0.29); } // make wrist go to that specific angle
    public void openClaw() { ClawTargetPosition = Constants.ClawOpenPosition; }
    public void closeClaw() { ClawTargetPosition = Constants.ClawClosedPosition; }

    /*
    public void toggleClaw() {
        if (ClawTargetPosition < 70) ClawTargetPosition = 90;
        else ClawTargetPosition = 0;
    }
    */

}
