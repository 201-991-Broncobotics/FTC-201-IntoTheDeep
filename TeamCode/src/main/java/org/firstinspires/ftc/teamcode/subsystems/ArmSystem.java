package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.FrameRateStabilizer;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.TelemetryLogger;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;


/*
Basic Summary:

ArmSystem is setup in a few sections:
1. ArmSystem - which initializes the arm
2. controlArmInTeleOp - which has some of the controller inputs (the rest are instant commands in AdvancedTeleOp)
     - It also has the auto aiming feature
3. updateClawArm - which moves the Arm to whatever the current target position is
     - It times the framerate for the entire code
     - Makes sure that the targeted arm position is within the limits
     - It backpedals the extension when needed
     - Runs the PIDs for the extension and pivot
     - Controls the claw and wrist
     - And has the telemetry for the majority code
4. tunePIDsWithController - Allows editing the PID variables while the robot is driving using the driver d-pad
5. Methods for getting the current exact or target position of the arm
6. Methods for setting the extension and pivot targets in multiple way:
     - Directly just sets the targets and makes sure they are within the limits
     - ToPoint allows setting the targets by specifying a claw height and forward distance relative to the robot
     - FieldCoord activates the field coord hold which attempts to hold the claw at a specific field coord regardless
          of the robots movement
7. Presets
8. Claw and Wrist Methods
9. Auton Methods for controlling arm with roadrunner
     - A way of running a preset or control method at a specified time in the future
     - A way for roadrunner to use that method
 */

/**
 * This subsystem is designed to control a pivot and extension arm with a wrist and claw by using
 * motion profiling. It also contains all of the presets, controls, telemetry, and auton actions for the arm.
 */
public class ArmSystem extends SubsystemBase {

    private DcMotorEx Pivot, ExtensionF, ExtensionB, PivotEncoder;
    private final Servo Wrist;
    private final CRServo Claw;
    private final PIDController PivotPID, ExtensionPID;

    private boolean PushForMaxExtension = false; // because the pid isn't accurate enough

    private double WristTargetAngle, PivotTargetAngle, ExtensionTargetLength;

    private double CurrentPivotAngleZero = 0, CurrentExtensionLengthZero = 0, CurrentPivotAngleDirectZero = 0;
    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360 - CurrentPivotAngleZero; // only using encoder of pivot motor
    public DoubleSupplier CurrentPivotAngleDirect = () -> PivotEncoder.getCurrentPosition() / Constants.encoderResolution * 360 - CurrentPivotAngleDirectZero; // rev encoder directly on pivot axle
    public DoubleSupplier CurrentExtensionLength = () -> ((ExtensionF.getCurrentPosition() / 384.5) * 360 + CurrentPivotAngleDirect.getAsDouble()) / Constants.SpoolDegreesToMaxExtension * 696 - CurrentExtensionLengthZero;
    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving
    private boolean slowDownExtensionAcceleration = false, ignoreAcceleration = false;

    ElapsedTime ArmLoopTimer, CommandFrameTime, PIDButtonPressTime, runTime, resetArmAlignmentHoldTimer, WristServoTimer, UnStallExtensionTimer;
    private double LastExtensionTarget = 0;
    public double FrameRate = 1, ArmLoopTime = 0;
    private ArrayList<Double> FrameRateCache = new ArrayList<Double>() ;

    Telemetry telemetry;
    private boolean telemetryEnabled = false;


    private double CurrentPivotAngleInst = 0, CurrentExtensionLengthInst = 0; // this speeds up the code a lot by only checking sensors one per update

    private double lastWristAngle;
    private double ClawServoPower = 0;
    private boolean keepClawSpinning = false, cameraToggle, activeExtensionReset = false, readyToResetArm = false;

    private int CurrentlyReadyPreset = 0; // allows pressing a preset button twice to complete the second part of its action

    private Vector2d FieldCoordHoldPos = new Vector2d(0, 0);
    private double FieldCoordHoldHeight = 0;

    private boolean needToSetStartCoord = true;

    // HuskyLensCamera camera;

    private double StartPivotZero, StartPivotDirectZero;

    private boolean OrthogonalMode = false, AutoAimCurrentlyExtending = false, autoAiming = false;

    private final FrameRateStabilizer frameRateStabilizer;

    private double EmergencyExtensionPowerRatio = 1; // for gradually releasing power to extension when over current
    public double StartTime = 0;

    private double LastPivotPower = 0, LastExtensionPower = 0, LastClawPower = 0;



    public ArmSystem(HardwareMap map, Telemetry inputTelemetry) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        ExtensionF = map.get(DcMotorEx.class, "ExtensionF");
        ExtensionB = map.get(DcMotorEx.class, "ExtensionB");
        PivotEncoder = map.get(DcMotorEx.class, "PivotEncoder");

        ExtensionF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PivotEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Claw = map.get(CRServo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
        WristTargetAngle = 0;
        lastWristAngle = WristTargetAngle;
        if (Settings.ArmSystemSettings.WristServoReversed) Wrist.setDirection(Servo.Direction.REVERSE);
        else Wrist.setDirection(Servo.Direction.FORWARD);

        SubsystemData.HoldClawFieldPos = false; // makes sure this is off on startup

        CurrentPivotAngleZero = CurrentPivotAngle.getAsDouble() - CurrentPivotAngleZero;
        CurrentPivotAngleDirectZero = CurrentPivotAngleDirect.getAsDouble() - CurrentPivotAngleDirectZero;

        // camera = new HuskyLensCamera(map);

        // Timers
        CommandFrameTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // this is how fast the entire code updates / loop time of command scheduler
        FrameRate = 1 / (CommandFrameTime.time() / 1000.0);
        PIDButtonPressTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ArmLoopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // never resets, mainly for auton
        resetArmAlignmentHoldTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // resets extension if resetArm is held
        WristServoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // stupid axon servo stops working correctly if told to go to a position while it is already going to a position
        UnStallExtensionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // stop extension motors from nearly catching fire again

        telemetry = inputTelemetry;

        PivotPID = new PIDController(Settings.ArmSystemSettings.PivotReference.kP, Settings.ArmSystemSettings.PivotReference.kI, Settings.ArmSystemSettings.PivotReference.kD, CurrentPivotAngle);
                //.setPositionLimiter(0, Settings.ArmSystemSettings.pivotMaxAngle)
                //.setSpeedLimiter(Settings.PivotPIDVariables.maxSpeed, 0, 0)
                //.setTolerance(3);

        ExtensionPID = new PIDController(Settings.ArmSystemSettings.ExtensionReference.kP, Settings.ArmSystemSettings.ExtensionReference.kI, Settings.ArmSystemSettings.ExtensionReference.kD, CurrentExtensionLength);
                //.setPositionLimiter(0, Constants.extensionMaxLength)
                //.setTolerance(15);

        PivotPID.setSettingsTheSameAs(Settings.ArmSystemSettings.PivotReference);
        ExtensionPID.setSettingsTheSameAs(Settings.ArmSystemSettings.ExtensionReference);

        frameRateStabilizer = new FrameRateStabilizer(0.75, 20, 100);
        frameRateStabilizer.disable(); // makes sure it isn't enabled in auton

        List<LynxModule> allHubs = map.getAll(LynxModule.class); // makes code faster
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    public void resetAndPrepareArm() {
        StartTime = runTime.time();

        // reset Extension and Pivot to make sure the backlash doesn't get in the way
        ExtensionF.setPower(-0.3);
        ExtensionB.setPower(-0.3);
        Pivot.setPower(-0.4); // makes sure the backlash is always at its max so it can be compensated in the pivot zero
        functions.Sleep(500);
        ExtensionF.setPower(0);
        ExtensionB.setPower(0);
        Pivot.setPower(0);
        CurrentPivotAngleZero = Pivot.getCurrentPosition() / 5281.1 * 360 - Constants.PivotDownAngle + Settings.ArmSystemSettings.pivotMotorBacklash;
        CurrentPivotAngleDirectZero = PivotEncoder.getCurrentPosition() / Constants.encoderResolution * 360 - Constants.PivotDownAngle;
        CurrentExtensionLengthZero = ((ExtensionF.getCurrentPosition() / 384.5) * 360 + 0) / Constants.SpoolDegreesToMaxExtension * 696;
        StartPivotZero = CurrentPivotAngleZero;
        StartPivotDirectZero = CurrentPivotAngleDirectZero;

        readyToResetArm = false;
    }


    public void controlArmTeleOp() {
        GamepadEx gamepad = SubsystemData.operator;
        // Manual arm control when controllers active
        double joystickLeftY = Math.pow(gamepad.getLeftY(), 3); //Math.abs(gamepad.getLeftY()) * gamepad.getLeftY();
        double joystickLeftX = Math.pow(gamepad.getLeftX(), 3); //Math.abs(gamepad.getLeftX()) * gamepad.getLeftX();
        double joystickRightY = Math.pow(gamepad.getRightY(), 3); //Math.abs(gamepad.getRightY()) * gamepad.getRightY();
        double AutoAimTrigger = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (SubsystemData.operator.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            OrthogonalMode = true;
            SubsystemData.HoldClawFieldPos = true;
            if (needToSetStartCoord) {
                FieldCoordHoldPos = getTargetClawPose().position;
                needToSetStartCoord = false;
            }
        }

        // turn off huskylens thread when not in use cause its laggy
        if ((functions.inUse(joystickRightY) || functions.inUse(joystickLeftY) || functions.inUse(joystickLeftX)) && FrameRate > 2) {
            if (functions.inUse(joystickLeftY)) PushForMaxExtension = false;
            backPedalExtension = false;
            double ArmThrottle = 0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (CurrentPivotAngleInst < 15) slowDownExtensionAcceleration = true;

            if (OrthogonalMode) { // field coords control (one joystick controls height, the other controls forward and strafe)
                /* Orthogonal version
                Vector2d targetClawPos = getTargetClawPoint();
                moveArmToPoint(new Vector2d(
                        targetClawPos.x + Constants.maxManualClawSpeedHorizontal * joystickLeftY * ArmThrottle / FrameRate,
                        targetClawPos.y + Constants.maxManualClawSpeedVertical * -1 * joystickRightY * ArmThrottle / FrameRate));
                 */
                SubsystemData.HoldClawFieldPos = true;
                if (needToSetStartCoord) {
                    FieldCoordHoldPos = getTargetClawPose().position;
                    needToSetStartCoord = false;
                }
                FieldCoordHoldPos = new Vector2d(
                        FieldCoordHoldPos.x + (Settings.ArmSystemSettings.maxManualClawSpeedHorizontal / 25.4) * joystickLeftX * ArmThrottle / FrameRate,
                        FieldCoordHoldPos.y + (Settings.ArmSystemSettings.maxManualClawSpeedHorizontal / 25.4) * joystickLeftY * ArmThrottle / FrameRate);
                FieldCoordHoldHeight = FieldCoordHoldHeight + Settings.ArmSystemSettings.maxManualClawSpeedVertical * -1 * joystickRightY * ArmThrottle / FrameRate;

            } else { // direct control (one joystick controls pivot, the other controls extension)
                SubsystemData.HoldClawFieldPos = false;
                double pivotThrottle = 1 - (1 - Settings.ArmSystemSettings.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength);
                moveArmDirectly(
                        PivotTargetAngle + Settings.ArmSystemSettings.maxManualPivotSpeed * -1 * joystickRightY * ArmThrottle * pivotThrottle / FrameRate,
                        ExtensionTargetLength + Settings.ArmSystemSettings.maxManualExtensionSpeed * joystickLeftY * ArmThrottle / FrameRate);
                //SubsystemData.HoldClawFieldPos = false;

                // operator can control turn when not auto aiming and driver isn't turning
                if (Math.abs(gamepad.getLeftX()) > 0.75 && !SubsystemData.HoldClawFieldPos && CurrentExtensionLengthInst > 100 && CurrentPivotAngleInst < 40) {
                    SubsystemData.OperatorTurningPower = -1 * Settings.ArmSystemSettings.OperatorTurnOverridePower * gamepad.getLeftX();
                } else {
                    SubsystemData.OperatorTurningPower = 0;
                    // SubsystemData.OverrideDrivetrainRotation = false;
                }
            }

        } else {
            SubsystemData.OperatorTurningPower = 0;
            autoAiming = functions.inUse(AutoAimTrigger);
        }


        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && resetArmAlignmentHoldTimer.time() > 1000) {
            activeExtensionReset = true; // keep retracting extension past limits until button is unpressed and then reset extension length
        } else if (!gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            resetArmAlignmentHoldTimer.reset();
            activeExtensionReset = false;
        }

        if (SubsystemData.operator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            openClaw();
        } else if (SubsystemData.operator.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            closeClaw();
        } else if (keepClawSpinning) {
            slowCloseClaw();
        } else stopClaw();

        updateClawArm();

        /*
        if (SubsystemData.inTeleOp) {
            frameRateStabilizer.Enabled = Settings.FrameRateStabilizerEditor.FrameRateStabilizerEnabled;
            frameRateStabilizer.lowerFrameRateRatio = Settings.FrameRateStabilizerEditor.FrameRateStabilizerRatio;
            frameRateStabilizer.maxFramesToAverage = Settings.FrameRateStabilizerEditor.FrameRateStabilizerMaxFrames;
            frameRateStabilizer.minimumFramesToAverage = Settings.FrameRateStabilizerEditor.FrameRateStabilizerMinFrames;
            frameRateStabilizer.maxWait = Settings.FrameRateStabilizerEditor.FrameRateStabilizerMaxWait;

            frameRateStabilizer.stabilize(); // yes this does slow down the code but I think it may make the pids more accurate
        }

         */

    }



    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly
        ArmLoopTimer.reset(); // time it takes the arm for 1 update
        FrameRateCache.add(1 / (CommandFrameTime.time() / 1000.0));
        if (FrameRateCache.size() > 4) FrameRateCache.remove(0);
        FrameRate = FrameRateCache.stream().mapToDouble(d -> d).average().orElse(1.0); // frame rate of the entire code averaged
        SubsystemData.FrameRate = FrameRate;
        CommandFrameTime.reset();

        double RunTimeInSeconds = functions.round((runTime.time() - StartTime) / 1000.0, 1);
        if (!SubsystemData.IMUWorking) {
            if (RunTimeInSeconds - Math.floor(RunTimeInSeconds) < 0.7) telemetry.addLine("##########################\n IMU HAS STOPPED RESPONDING \n##########################");
            else telemetry.addLine(" \n \n ");
        }
        int minutes = (int) Math.floor(RunTimeInSeconds / 60);
        telemetry.addLine("Run Time - " + minutes + ":" + functions.round(RunTimeInSeconds - minutes * 60, 1));

        if (!SubsystemData.LocalizationCoordsAligned[0] && !SubsystemData.LocalizationCoordsAligned[1]) telemetry.addLine("Auto Driving is not locked in");
        else if (!SubsystemData.LocalizationCoordsAligned[0]) telemetry.addLine("Auto Driving: X needs to be aligned");
        else if (!SubsystemData.LocalizationCoordsAligned[1]) telemetry.addLine("Auto Driving: Y needs to be aligned");
        telemetry.addData("Code FrameRate", FrameRate);
        telemetry.addData("Code Loop Time", 1 / FrameRate);
        //telemetry.addData("Arm System Loop Time", ArmLoopTime);
        //telemetry.addData("Drive System Loop Time", SubsystemData.DrivetrainLoopTime);
        //telemetry.addData("HuskyLens Loop Time", SubsystemData.HuskyLensLoopTime);
        //telemetry.addData("HuskyLens Thread Loop Time", SubsystemData.HuskyLensThreadLoopTime);
        //telemetry.addLine(" ");
        // very important that this is in telemetry or the encoder doesn't work correctly, WHY? idk
        telemetry.addData("Schrödinger's Encoder", SubsystemData.brokenDiffyEncoder.getCurrentPosition());


        // this speeds up the code a lot by only checking sensors once per update
        CurrentPivotAngleInst = CurrentPivotAngleDirect.getAsDouble(); // uses the more accurate direct encoder only when below 70 degrees to prevent the backlash from affecting the arm
        if (CurrentPivotAngleInst > 70) CurrentPivotAngleInst = CurrentPivotAngle.getAsDouble();

        if (CurrentPivotAngleInst < Constants.PivotDownAngle) CurrentPivotAngleInst = Constants.PivotDownAngle; // caps pivot angle values to within range
        CurrentExtensionLengthInst = CurrentExtensionLength.getAsDouble();

        // send current extension length percent to the driveCommand so speed can be limited
        SubsystemData.DriveCurrentExtensionLengthPercent = CurrentExtensionLengthInst / Constants.extensionMaxLength;

        // Run any delayed auton methods that are ready to run
        runAnyPreparedMethods();


        //telemetry.addData("Point 1:", ArmLoopTimer.time() - LastArmLoopTime);
        //LastArmLoopTime = ArmLoopTimer.time();


        /*

        // AUTO AIMING
        if (autoAiming) { // AUTO AIM
            telemetry.addLine("Auto Aiming Active");
            slowDownExtensionAcceleration = false;
            OrthogonalMode = false;
            SubsystemData.HoldClawFieldPos = false;
            if (!cameraToggle) {
                cameraToggle = true;
                camera.StartHuskyLensThread();
                setWristToRaisedFloor();
                if (CurrentExtensionLengthInst < 100 && CurrentPivotAngleInst < 15) {
                    AutoAimCurrentlyExtending = true;
                    ExtensionTargetLength = 250;
                } else {
                    backPedalExtension = true;
                    moveArmDirectly(0, 0);
                }
            }
            if (AutoAimCurrentlyExtending && CurrentExtensionLengthInst > 250 && !(CurrentPivotAngleInst > 15)) {
                AutoAimCurrentlyExtending = false;
            }

            camera.ScanForSample();

            if (SubsystemData.CameraSeesValidObject) {
                if (AutoAimCurrentlyExtending) { // the point of this is to extend the extension when auto aim is first pressed but if it sees something before it fully extends, it will stop
                    ExtensionTargetLength = CurrentExtensionLengthInst;
                    AutoAimCurrentlyExtending = false;
                }

                double ExtensionInput = (SubsystemData.CameraTargetPixelsY / 120) / FrameRate;
                double ExtensionPower = -1 * (Settings.ArmSystemSettings.maxCameraTargetingSpeed * ExtensionInput + Settings.ArmSystemSettings.maxCameraTargetingSpeedSquared * Math.abs(ExtensionInput) * ExtensionInput);
                moveArmDirectly(0, ExtensionTargetLength + ExtensionPower);

                // slows down camera auto aim turn speed the further extension is extended
                double CameraTargetingTurnThrottle = (Constants.pivotAxleOffset + Constants.retractedExtensionLength) / (Constants.pivotAxleOffset + Constants.retractedExtensionLength + CurrentExtensionLengthInst);
                double AutoAimHeadingChange = CameraTargetingTurnThrottle * (SubsystemData.CameraTargetPixelsX / 160);
                SubsystemData.OperatorTurningPower = Settings.ArmSystemSettings.OperatorDriveTurnPower * AutoAimHeadingChange;
            }

        } else if (cameraToggle) { // turn off huskylens thread when not in use cause its laggy
            cameraToggle = false;
            camera.EndHuskyLensThread();
        }

         */


        // HOLD CLAW AT A FIELD COORD

        if (SubsystemData.HoldClawFieldPos) {
            slowDownExtensionAcceleration = false;

            // Make Hold Pos stay within the robot's limits
            double TargetPosDistance = Math.hypot(FieldCoordHoldPos.x - SubsystemData.CurrentRobotPose.position.x, FieldCoordHoldPos.y - SubsystemData.CurrentRobotPose.position.y);
            double TargetPosDirection = Math.atan2(FieldCoordHoldPos.y - SubsystemData.CurrentRobotPose.position.y, FieldCoordHoldPos.x - SubsystemData.CurrentRobotPose.position.x); // in radians

            if (Settings.ArmSystemSettings.ObeyHorizontalLimit && TargetPosDistance > (Constants.freeHorizontalExpansion + Constants.retractedExtensionLength) / 25.4) {
                TargetPosDistance =  (Constants.freeHorizontalExpansion + Constants.retractedExtensionLength) / 25.4;
            } else if (TargetPosDistance < (Constants.retractedExtensionLength + Constants.pivotAxleOffset + 50) / 25.4) {
                TargetPosDistance = (Constants.retractedExtensionLength + Constants.pivotAxleOffset + 50) / 25.4;
            }

            FieldCoordHoldPos = new Vector2d(TargetPosDistance * Math.cos(TargetPosDirection) + SubsystemData.CurrentRobotPose.position.x, TargetPosDistance * Math.sin(TargetPosDirection) + SubsystemData.CurrentRobotPose.position.y);

            if (FieldCoordHoldHeight > (Constants.extensionMaxLength + Constants.retractedExtensionLength + Constants.pivotAxleHeight)) {
                FieldCoordHoldHeight = (Constants.extensionMaxLength + Constants.retractedExtensionLength + Constants.pivotAxleHeight);
            } else if (FieldCoordHoldHeight < Constants.pivotAxleHeight / 25.4) {
                FieldCoordHoldHeight = Constants.pivotAxleHeight / 25.4;
            }

            holdClawAtFieldCoordinate(FieldCoordHoldPos, FieldCoordHoldHeight);
        } else {
            needToSetStartCoord = true;
            SubsystemData.OverrideDrivetrainRotation = false;
        }



        // Make sure the targets are within the limits
        if (PivotTargetAngle > Settings.ArmSystemSettings.pivotMaxAngle) PivotTargetAngle = Settings.ArmSystemSettings.pivotMaxAngle;
        else if (PivotTargetAngle < Constants.PivotDownAngle) PivotTargetAngle = Constants.PivotDownAngle;
        if (ExtensionTargetLength > Constants.extensionMaxLength + 2) ExtensionTargetLength = Constants.extensionMaxLength + 2;
        else if (ExtensionTargetLength < -2) ExtensionTargetLength = -2;

        if (Settings.ArmSystemSettings.ObeyHorizontalLimit && ExtensionTargetLength * Math.cos(Math.toRadians(CurrentPivotAngleInst)) > Constants.freeHorizontalExpansion - 50) {
            ExtensionTargetLength = Constants.freeHorizontalExpansion - 50; // limiter against continuing to power the extension motors outside of the horizontal limit (there are a lot of extension limiters)
        }



        // LINEAR SLIDE BENDING IS STUPID
        double linearSlideBendCompensation = -1 * Settings.ArmSystemSettings.LinearSlideBend * (CurrentExtensionLengthInst / Constants.extensionMaxLength) * Math.cos(Math.toRadians(PivotTargetAngle));


        if (slowDownExtensionAcceleration) {
            Settings.ArmSystemSettings.ExtensionReference.maxAcceleration = Settings.ArmSystemSettings.ExtensionSubMaxAcceleration;
            Settings.ArmSystemSettings.ExtensionReference.maxDeceleration = Settings.ArmSystemSettings.ExtensionSubMaxDeceleration;
        } else {
            Settings.ArmSystemSettings.ExtensionReference.maxAcceleration = Settings.ArmSystemSettings.ExtensionNormalMaxAcceleration;
            Settings.ArmSystemSettings.ExtensionReference.maxDeceleration = Settings.ArmSystemSettings.ExtensionNormalMaxDeceleration;
        }

        // Makes sure any changes to pid variables get applied to the actual pids
        PivotPID.setSettingsTheSameAs(Settings.ArmSystemSettings.PivotReference);
        ExtensionPID.setSettingsTheSameAs(Settings.ArmSystemSettings.ExtensionReference);


        // BACKPEDALING

        // The purpose of Backpedal is to retract the extension when the pivot needs to move a lot so that the arm is less likely to hit something and the pivot can rotate faster
        // if backpedal is enabled and the angle already traveled is less than half of the total angle that needs to be traversed

        // backpedals only if the pivot needs to move more than 10 degrees
        if (Math.abs(PivotTargetAngle - CurrentPivotAngleInst) < 10) backPedalExtension = false;

        if (backPedalExtension) {
            if (CurrentExtensionLengthInst < 150 || CurrentPivotAngleInst > Settings.ArmSystemSettings.PivotBacklashMaxAngle) { // if extension is already retracted
                PivotPID.setTarget(PivotTargetAngle); // move pivot
                ExtensionPID.setTarget(25);
            } else ExtensionPID.setTarget(25); // otherwise stop changing the pivot target in the pid (hold the current pivot angle) and retract extension
        } else {
            PivotPID.setTarget(PivotTargetAngle + linearSlideBendCompensation);

            // possibly another horizontal extension limiter
            if (CurrentPivotAngleInst < 70) {
                ExtensionPID.maxPosition = Math.min(Constants.extensionMaxLength, Constants.freeHorizontalExpansion / Math.cos(Math.toRadians(CurrentPivotAngleInst)));
            }

            if (ignoreAcceleration) {
                ExtensionPID.jumpSetTarget(ExtensionTargetLength);
                ignoreAcceleration = false;
            } else {
                ExtensionPID.setTarget(ExtensionTargetLength);
            }
        }



        // PIVOT

        PivotPID.setPercentMaxSpeed(1 - (1 - Settings.ArmSystemSettings.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength));
        // set pivot power to pid value + the amount of power needed to counteract gravity at the current pivot angle and current extension length
        double PivotPIDPower = PivotPID.getPower();
        double PivotPower = PivotPIDPower
                + ((Settings.ArmSystemSettings.pivotExtendedGravityPower - Settings.ArmSystemSettings.pivotRetractedGravityPower) / Constants.extensionMaxLength * CurrentExtensionLengthInst + Settings.ArmSystemSettings.pivotRetractedGravityPower) * Math.cos(Math.toRadians(CurrentPivotAngleInst))
                + (-1 * Settings.ArmSystemSettings.PivotAccelerationDampeningPower * SubsystemData.CurrentForwardAcceleration * (CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.sin(CurrentPivotAngleInst));
        // if (Math.abs(PivotPower) > 0.75) PivotPower = Math.signum(PivotPower) * 0.75; // sets max pivot power
        if ((CurrentPivotAngleInst < 2 + Constants.PivotDownAngle && PivotTargetAngle < 2 + Constants.PivotDownAngle && ExtensionTargetLength < 75) || FrameRate < 2) { // stop pivot if it is resting on the mechanical stop or if the framerate is less than 2
            PivotPower = 0;
        }

        if (Math.abs(LastPivotPower - PivotPower) >= Settings.ArmMotorReadDifference || (PivotPower == 0 && !(LastPivotPower == 0))) {
            Pivot.setPower(PivotPower);
            LastPivotPower = PivotPower;
        }




        // EXTENSION

        double ExtensionPIDPower = ExtensionPID.getPower();
        double ExtensionPower = ExtensionPIDPower + Math.sin(Math.toRadians(CurrentPivotAngleInst)) * Settings.ArmSystemSettings.extensionGravityPower;
        if (FrameRate < 2) ExtensionPower = 0; // emergency stop extension if framerate is less than 2

        if (Settings.ArmSystemSettings.ObeyHorizontalLimit && CurrentExtensionLengthInst * Math.cos(Math.toRadians(CurrentPivotAngleInst)) > Constants.freeHorizontalExpansion + 100 && ExtensionPower > 0) {
            ExtensionPower = 0; // limiter against continuing to power the extension motors outside of the horizontal limit (there are a lot of extension limiters)
            PushForMaxExtension = false;
        }

        if (activeExtensionReset) { // reset extension length
            ExtensionF.setPower(-0.3);
            ExtensionB.setPower(-0.3);
            Pivot.setPower(-0.4);
            PivotTargetAngle = 0;
            ExtensionTargetLength = -5;
            readyToResetArm = true;
        } else if (PushForMaxExtension && CurrentExtensionLengthInst > 630 && CurrentPivotAngleInst > 75) {
            ExtensionF.setPower(0.25);
            ExtensionB.setPower(0.25);
        } else if (readyToResetArm) {
            CurrentPivotAngleZero = Pivot.getCurrentPosition() / 5281.1 * 360 - Constants.PivotDownAngle + Settings.ArmSystemSettings.pivotMotorBacklash;
            CurrentPivotAngleDirectZero = PivotEncoder.getCurrentPosition() / Constants.encoderResolution * 360 - Constants.PivotDownAngle;
            CurrentExtensionLengthZero = ((ExtensionF.getCurrentPosition() / 384.5) * 360 + 0) / Constants.SpoolDegreesToMaxExtension * 696;
            readyToResetArm = false;
        } else {

            if (Math.abs(LastExtensionPower - ExtensionPower * EmergencyExtensionPowerRatio) >= Settings.ArmMotorReadDifference || (ExtensionPower * EmergencyExtensionPowerRatio == 0 && !(LastExtensionPower == 0))) {
                ExtensionF.setPower(ExtensionPower * EmergencyExtensionPowerRatio);
                ExtensionB.setPower(ExtensionPower * EmergencyExtensionPowerRatio);
                LastExtensionPower = ExtensionPower * EmergencyExtensionPowerRatio;
            }

        }


        // This should stop the extension motors if they have high current and their target has not changed for awhile to prevent them from catching fire
        if (!(LastExtensionTarget == ExtensionTargetLength)) {
            UnStallExtensionTimer.reset();
            EmergencyExtensionPowerRatio = 1;
        }
        LastExtensionTarget = ExtensionTargetLength;
        if (UnStallExtensionTimer.time() > Settings.ArmSystemSettings.EmergencyExtensionPowerReleaseTimeout && ExtensionF.getCurrent(CurrentUnit.AMPS) > 0.5) { // 30 second timeout
            if (EmergencyExtensionPowerRatio > 0) EmergencyExtensionPowerRatio -= Settings.ArmSystemSettings.EmergencyExtensionPowerReleaseSpeed / FrameRate;
            else if (EmergencyExtensionPowerRatio < 0) EmergencyExtensionPowerRatio = 0;
        }


        // WRIST AND CLAW

        if (WristTargetAngle > 90) setWrist(WristTargetAngle + linearSlideBendCompensation);
        else setWrist(WristTargetAngle);

        if (Math.abs(LastClawPower - ClawServoPower) >= Settings.ArmMotorReadDifference || (ClawServoPower == 0 && !(LastClawPower == 0))) {
            Claw.setPower(ClawServoPower);
            LastClawPower = ClawServoPower;
        }


        telemetry.addLine("Robot Pose: " + functions.TilePoseAsString(SubsystemData.CurrentRobotPose));
        telemetry.addLine("Robot Velocity X:" +
                functions.round(SubsystemData.RobotVelocity.linearVel.x, 2) + " Y:" +
                functions.round(SubsystemData.RobotVelocity.linearVel.y, 2));
        /*
        telemetry.addData("Pedro Heading", Math.toDegrees(SubsystemData.CurrentPedroPose.getHeading()));
        telemetry.addLine("Pedro Pose (in) X: " +
                functions.round(SubsystemData.CurrentPedroPose.getX(), 2) + " Y: " +
                functions.round(SubsystemData.CurrentPedroPose.getY(), 2));
        telemetry.addLine("Pedro Velocity X: " +
                functions.round(SubsystemData.CurrentPedroVelocity.getXComponent(), 2) + " Y: " +
                functions.round(SubsystemData.CurrentPedroVelocity.getYComponent(), 2));

         */


        if (telemetryEnabled) { // a lot of telemetry slows the code down so I made it toggleable
            telemetry.addLine(" ");
            telemetry.addData("ExtensionPID max Position", ExtensionPID.maxPosition);
            /*
            telemetry.addData("Forward Acceleration", SubsystemData.CurrentForwardAcceleration);
            telemetry.addLine(" ");
            telemetry.addData("FrameRate Stabilization Enabled", frameRateStabilizer.isEnabled());
            telemetry.addData("FrameRate Stabilization average FrameRate", frameRateStabilizer.getAverageFrameRate());
            telemetry.addData("FrameRate Stabilization number of saved times", frameRateStabilizer.getCacheSize());
            telemetry.addData("FrameRate Stabilization wait time", frameRateStabilizer.getCurrentWaitTime());
            telemetry.addData("FrameRate Stabilization min max difference", frameRateStabilizer.getMinMaxTimeDifference());
            telemetry.addLine(" ");

             */
            telemetry.addData("Is Auto Driving", SubsystemData.AutoDriving);
            switch (SubsystemData.CurrentPathSetting) {
                case 0: telemetry.addData("Current Auto Driving Selection", "Submersible"); break;
                case 1: telemetry.addData("Current Auto Driving Selection", "Human Player"); break;
                case 2: telemetry.addData("Current Auto Driving Selection", "Chamber"); break;
                case 3: telemetry.addData("Current Auto Driving Selection", "Basket"); break;
            }
            telemetry.addData("Current Auto Driving Path", SubsystemData.CurrentPathSetting);
            telemetry.addData("Auto Driving Power", SubsystemData.AutoDrivingPower);
            telemetry.addData("Extension Max Power Percent", EmergencyExtensionPowerRatio);
            telemetry.addData("Roller Claw Power", ClawServoPower);
            //telemetry.addData("Servo Position", Wrist.getPosition());
            //telemetry.addData("Servo set angle", lastWristAngle);
            //telemetry.addData("Wrist Target Angle", WristTargetAngle);
            telemetry.addData("Heading target", SubsystemData.HeadHoldTarget);
            if (SubsystemData.HoldClawFieldPos) {
                telemetry.addLine("Field Hold Pose (tiles) X: " +
                        functions.round(FieldCoordHoldPos.x / Constants.tileLength, 3) + " Y: " +
                        functions.round(FieldCoordHoldPos.y / Constants.tileLength, 3));
                telemetry.addData("Field Hold Height", FieldCoordHoldHeight);
            } else telemetry.addLine("Not Field Holding");
            Vector2d CurrentClawPosition = getCurrentClawPoint(); // avoiding calling this method twice
            telemetry.addLine("Claw Point X: " + functions.round(CurrentClawPosition.x, 2) + " Y: " + functions.round(CurrentClawPosition.y, 2));
            Vector2d TargetClawPosition = getTargetClawPoint(); // avoiding calling this method twice
            telemetry.addLine("Target Claw Point X: " + functions.round(TargetClawPosition.x, 2) + " Y: " + functions.round(TargetClawPosition.y, 2));
            Pose2d CurrentClawPose = getCurrentClawPose(); // avoiding calling this method twice
            telemetry.addLine("Current Claw Pose X: " + functions.round(CurrentClawPose.position.x, 4) + " Y: " + functions.round(CurrentClawPose.position.y, 4));
            telemetry.addLine(" ");
            telemetry.addData("Extension Target Length", ExtensionTargetLength);
            telemetry.addData("Extension Current Length", CurrentExtensionLengthInst);
            telemetry.addData("Pivot Target Angle", PivotTargetAngle + linearSlideBendCompensation);
            telemetry.addData("Pivot Current Angle Motor", CurrentPivotAngle.getAsDouble());
            telemetry.addData("Pivot Current Angle Direct", CurrentPivotAngleDirect.getAsDouble());
            telemetry.addData("Pivot Current Angle in use", CurrentPivotAngleInst);
            telemetry.addLine(" ");
            telemetry.addData("Distance from Pivot Start Zero", CurrentPivotAngleZero - StartPivotZero);
            telemetry.addData("Distance from Pivot Encoder Start Zero", CurrentPivotAngleDirectZero - StartPivotDirectZero);
            //telemetry.addData("Pivot PID Power:", PivotPIDPower);
            //telemetry.addData("Extension PID Power:", ExtensionPIDPower);
            //telemetry.addData("Extension Difference", ExtensionTargetLength - CurrentExtensionLengthInst);
            //telemetry.addData("Pivot Difference", PivotTargetAngle + linearSlideBendCompensation - CurrentPivotAngleInst);

            /*
            telemetry.addLine(" ");
            if (SubsystemData.HuskyLensConnected) telemetry.addLine("HuskyLens Active");
            else telemetry.addLine("HuskyLens not responding");
            telemetry.addLine(" ");
            if (SubsystemData.CameraSeesValidObject) {
                switch (SubsystemData.CameraTargetId) {
                    case 1: telemetry.addLine("Targeting Red");
                    case 2: telemetry.addLine("Targeting Yellow");
                    case 3: telemetry.addLine("Targeting Blue");
                }
                telemetry.addData("X Pixels from Target", SubsystemData.CameraTargetPixelsX);
                telemetry.addData("Y Pixels from Target", SubsystemData.CameraTargetPixelsY);
                telemetry.addLine(" ");
            }
            HuskyLens.Block[] VisionResults = SubsystemData.Vision;
            telemetry.addData("HuskyLens block count", VisionResults.length);
            for (HuskyLens.Block value : VisionResults) {
                telemetry.addLine("ID:" + (value.id) + " x:" + (value.x) + " y:" + (value.y) + // Id, center X, center Y
                        " h:" + (value.height) + " w:" + (value.width)); // height, width,  + " ox" + (value.left) + " oy" + (value.top)  origin X, Origin Y
            }

            telemetry.addLine(" \n \n \n \n \n \n \n "); // adds spacing so I can actually read the huskylens data without it scrolling

             */
        }

        // Gives all the times that I logged inside parts of the code so I can optimise
        TelemetryLogger.addLogsToTelemetry(telemetry);

        ArmLoopTime = ArmLoopTimer.time(); // updates how long the arm loop took to run
    }

    public void prepareServosForAuton() {
        setWristToBack();
        closeClaw();
        Claw.setPower(ClawServoPower);
        reallySlowCloseClaw();
        Claw.setPower(ClawServoPower);

        Wrist.setPosition(1 - (Settings.ArmSystemSettings.WristServoRatio * ((WristTargetAngle + Settings.ArmSystemSettings.WristServoOffset) / 360.0))); // (48/40.0) * 1.33
        lastWristAngle = WristTargetAngle;
        Claw.setPower(ClawServoPower);
    }


    private Vector2d getCurrentClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((CurrentExtensionLengthInst + Constants.retractedExtensionLength + Constants.wristLength) * Math.cos(Math.toRadians(CurrentPivotAngleInst)),
                (CurrentExtensionLengthInst + Constants.retractedExtensionLength + Constants.wristLength) * Math.sin(Math.toRadians(CurrentPivotAngleInst)));
    }


    private Vector2d getTargetClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((ExtensionTargetLength + Constants.retractedExtensionLength + Constants.wristLength) * Math.cos(Math.toRadians(PivotTargetAngle)),
                (ExtensionTargetLength + Constants.retractedExtensionLength + Constants.wristLength) * Math.sin(Math.toRadians(PivotTargetAngle)));
    }


    public Pose2d getCurrentClawPose() { // Note: HEADING IN RADIANS and field centric in inches
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d CurrentClawPoint = getCurrentClawPoint(); // Claw Position relative to pivot axle
        return new Pose2d((new Vector2d(
                (CurrentClawPoint.x + Constants.pivotAxleOffset) / 25.4 * Math.cos(CurrentPose.heading.toDouble()),
                (CurrentClawPoint.x + Constants.pivotAxleOffset) / 25.4 * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public Pose2d getTargetClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d TargetClawPoint = getTargetClawPoint(); // Claw Target Position relative to pivot axle
        return new Pose2d((new Vector2d(
                (TargetClawPoint.x + Constants.pivotAxleOffset) / 25.4 * Math.cos(CurrentPose.heading.toDouble()),
                (TargetClawPoint.x + Constants.pivotAxleOffset) / 25.4 * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public double getCurrentClawHeight() { // FROM FIELD TILES instead of from pivot axle, IN MILLIMETERS instead of inches just because
        // this has to be separate from pose since I don't want to create data types or use lists
        return Math.sin(Math.toRadians(CurrentPivotAngleInst)) *
                (CurrentExtensionLengthInst + Constants.retractedExtensionLength) + Constants.pivotAxleHeight;
    }


    public double getTargetClawHeight() { // FROM FIELD TILES instead of from pivot axle
        // this has to be separate from pose since I don't want to create data types or use lists
        return Math.sin(Math.toRadians(PivotTargetAngle)) *
                (ExtensionTargetLength + Constants.retractedExtensionLength) + Constants.pivotAxleHeight;
    }




    // MOVE ARM METHODS AND PRESETS:
    private void moveArmDirectly(double pivotTarget, double extensionTarget) {
        PivotTargetAngle = pivotTarget;
        ExtensionTargetLength = extensionTarget;
        if (PivotTargetAngle < Constants.PivotDownAngle) PivotTargetAngle = Constants.PivotDownAngle; // prevent pivot from exceeding max positions
        else if (PivotTargetAngle > Settings.ArmSystemSettings.pivotMaxAngle) PivotTargetAngle = Settings.ArmSystemSettings.pivotMaxAngle;
        // extension get realigned because it skips a lot instead of locking it at max positions
        // if (ExtensionTargetLength < 0) ExtensionTargetLength = 0;
        // else if (ExtensionTargetLength > Constants.extensionMaxLength) ExtensionTargetLength = Constants.extensionMaxLength;
    }


    private void moveArmToPoint(Vector2d point) { // units:mm, makes pivot and extension work together to go to a set point relative to the pivot axle on the robot
        double Y = point.y;
        double X = point.x;

        // This horizontal extension limiter allows for the arm to at least go to the correct height if the target x is outside the expansion limit
        if (Settings.ArmSystemSettings.ObeyHorizontalLimit && X > Constants.freeHorizontalExpansion + Constants.retractedExtensionLength) X = Constants.freeHorizontalExpansion + Constants.retractedExtensionLength;
        moveArmDirectly(Math.toDegrees(Math.atan2(Y, X)), Math.hypot(X, Y) - Constants.retractedExtensionLength);
    }


    public void holdClawAtFieldCoordinate(Vector2d TargetClawPos, double TargetHeight) { // needs to be called constantly while in use, HEIGHT IS FROM FIELD TILES
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d DeltaPose = TargetClawPos.minus((CurrentPose.position).plus(new Vector2d(Settings.ArmSystemSettings.FieldCentricArmVelocityCompensation * SubsystemData.CorrectedRobotVelocity.linearVel.x, Settings.ArmSystemSettings.FieldCentricArmVelocityCompensation * SubsystemData.CorrectedRobotVelocity.linearVel.y))); // Vector of where the arm needs to go relative to the robot with the robot's velocity in mind

        double NeededExtensionLength = 25.4 * Math.hypot(DeltaPose.x, DeltaPose.y) - Constants.pivotAxleOffset - Constants.wristLength - 35; // mm

        // tell the drivetrain to point towards the target point
        SubsystemData.OverrideDrivetrainTargetHeading = Math.toDegrees(Math.atan2(DeltaPose.y, DeltaPose.x));
        SubsystemData.OverrideDrivetrainRotation = true;

        moveArmToPoint(new Vector2d(NeededExtensionLength, TargetHeight - Constants.pivotAxleHeight));
    }


    public void moveClawToFieldCoordinate(Vector2d TargetClawPos, double TargetHeight) { // needs to be called constantly while in use, HEIGHT IS FROM FIELD TILES
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d DeltaPose = TargetClawPos.minus(CurrentPose.position); // Vector of where the arm needs to go relative to the robot with the robot's velocity in mind

        double NeededExtensionLength = 25.4 * Math.hypot(DeltaPose.x, DeltaPose.y) - Constants.pivotAxleOffset - Constants.wristLength - 35; // mm

        moveArmToPoint(new Vector2d(NeededExtensionLength, TargetHeight - Constants.pivotAxleHeight));
    }


    // CurrentlyReadyPreset:
    // 0 = none
    // 1 = moveClawToTopBasket
    // 2 = moveClawToTopRung
    // 3 = moveClawToHumanPickup
    // 4 = moveClawIntoSubmersible


    public void resetArm() {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        PushForMaxExtension = false;
        OrthogonalMode = false;
        CurrentlyReadyPreset = 0;
        backPedalExtension = true;
        if (PivotTargetAngle > 40 || ExtensionTargetLength < 100 || !SubsystemData.inTeleOp) { // makes it so retracting in the submersible doesn't get the arm stuck
            PivotTargetAngle = 0;
            WristTargetAngle = 0;
        } else if (PivotTargetAngle < 40 && ExtensionTargetLength > 100 && WristTargetAngle > 150) WristTargetAngle = 150;
        ExtensionTargetLength = 0;

    }


    public void moveClawToTopBasket() {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        backPedalExtension = true;
        OrthogonalMode = false;
        PivotTargetAngle = 87;
        ExtensionTargetLength = 696;
        PushForMaxExtension = true;
        WristTargetAngle = 165;
        CurrentlyReadyPreset = 1;
    }


    public void moveClawToTopRung() {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        PushForMaxExtension = false;
        OrthogonalMode = false;
        if (CurrentlyReadyPreset == 2 && SubsystemData.inTeleOp) { // second action
            depositSpecimen();
            CurrentlyReadyPreset = 0;
        } else { // normal action
            backPedalExtension = true;
            PivotTargetAngle = Settings.ArmSystemSettings.ChamberPresetPivotAngle;
            ExtensionTargetLength = Settings.ArmSystemSettings.ChamberPresetExtensionLength;
            WristTargetAngle = 180;
            CurrentlyReadyPreset = 2;
        }
    }

    public void moveClawToTopRungAdjusted(double pivotModifier, double extensionModifier) {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        PushForMaxExtension = false;
        OrthogonalMode = false;
        backPedalExtension = true;
        PivotTargetAngle = Settings.ArmSystemSettings.ChamberPresetPivotAngle + pivotModifier;
        ExtensionTargetLength = Settings.ArmSystemSettings.ChamberPresetExtensionLength + extensionModifier;
        WristTargetAngle = 180;
        CurrentlyReadyPreset = 2;
    }

    public void moveClawToTopRungAuto() {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        PushForMaxExtension = false;
        OrthogonalMode = false;

        backPedalExtension = true;
        PivotTargetAngle = Settings.ArmSystemSettings.AutoChamberPresetPivotAngle;
        ExtensionTargetLength = Settings.ArmSystemSettings.AutoChamberPresetExtensionLength;
        WristTargetAngle = Settings.ArmSystemSettings.WristAutonChamberAngle;
    }


    public void depositSpecimen() { // onto a rung - this is separate so it is easier to code auton
        ignoreAcceleration = true;
        ExtensionTargetLength = CurrentExtensionLengthInst + 200;
    }


    public void moveClawToHumanPickup() {
        SubsystemData.HoldClawFieldPos = false;
        slowDownExtensionAcceleration = false;
        PushForMaxExtension = false;
        OrthogonalMode = false;
        if (CurrentlyReadyPreset == 3 && SubsystemData.inTeleOp) { // second action
            closeClaw();
            PivotTargetAngle = Settings.ArmSystemSettings.HumanPlayerPresetPivotAngle;
            ExtensionTargetLength = 100;
            CurrentlyReadyPreset = 0;
        } else {
            backPedalExtension = true;
            ExtensionTargetLength = 0;
            PivotTargetAngle = Settings.ArmSystemSettings.HumanPlayerPresetPivotAngle;
            WristTargetAngle = 0;
            CurrentlyReadyPreset = 3;
        }
    }


    public void moveClawIntoSubmersible() {
        CurrentlyReadyPreset = 4;
        slowDownExtensionAcceleration = false;
        backPedalExtension = true;
        OrthogonalMode = true;
        SubsystemData.HoldClawFieldPos = true;
        holdClawAtFieldCoordinate(new Vector2d(0, 0), 120);
        setWristToFloorPickup();
    }


    public void dropSamplePickup() { // the wrist pivots down to the ground before the claw closes

        // Doesn't work in autons for whatever reason
        if (CurrentPivotAngleInst < 30 || !SubsystemData.inTeleOp) {
            // only try to pickup sample if in the correct starting position, else move to the starting position
            if (WristTargetAngle > 135 || !SubsystemData.inTeleOp) { // execute pickup action
                closeClaw();
                setWristToFloorPickup();
                runMethodAfterSec("setWristToRaisedFloor", 0.6);
            } else { // move wrist to correct position
                closeClaw();
                setWristToFloorPickup();
                runMethodAfterSec("setWristToRaisedFloor", 1.5);
            }
        } // otherwise do nothing because the arm is completely in the wrong position
    }



    // WRIST AND CLAW METHODS

    // Gobilda torque servo: Wrist.setPosition(1.35 * (Angle / 360) + 0.29);

    public void setWrist(double Angle) { // make wrist go to that specific angle
        if (Settings.ArmSystemSettings.WristServoReversed) Wrist.setDirection(Servo.Direction.REVERSE);
        else Wrist.setDirection(Servo.Direction.FORWARD);
        if (!(Angle == lastWristAngle)) {
            Wrist.setPosition(1 - (Settings.ArmSystemSettings.WristServoRatio * ((Angle + Settings.ArmSystemSettings.WristServoOffset) / 360.0))); // (48/40.0) * 1.33
            lastWristAngle = Angle;
        }
    }

    public void setClaw(double power) {
        ClawServoPower = power;
    }
    public void openClaw() {
        ClawServoPower = Settings.ArmSystemSettings.ClawOpeningPower;
        keepClawSpinning = false;
    }
    public void closeClaw() {
        ClawServoPower = Settings.ArmSystemSettings.ClawClosingPower;
        keepClawSpinning = true;
    }
    public void slowCloseClaw() {
        ClawServoPower = Settings.ArmSystemSettings.ClawClosingSlowPower;
    }
    public void reallySlowCloseClaw() {
        ClawServoPower = Settings.ArmSystemSettings.ClawClosingReallySlowPower;
    }
    public void stopClaw() {
        ClawServoPower = 0;
        keepClawSpinning = false;
    }
    public void setWristToBack() { WristTargetAngle = 0; }
    public void setWristToStraight() { WristTargetAngle = 180; }
    public void setWristToRaisedFloor() { WristTargetAngle = 180; } // remember to update this value in the dropSamplePickup
    public void setWristToFloorPickup() { WristTargetAngle = Settings.ArmSystemSettings.WristFloorAngle; }
    public void setWristToAutoAngle() { WristTargetAngle = Settings.ArmSystemSettings.WristAutonChamberAngle; }
    public void toggleBetweenStraightAndFloor() {
        if (!(WristTargetAngle == 180 || WristTargetAngle == 170))
            if (CurrentPivotAngleInst > 45) setWristToStraight();
            else setWristToRaisedFloor();
        else setWristToFloorPickup();
    }


    public void toggleTelemetry() { telemetryEnabled = !telemetryEnabled; }



    // ROADRUNNER AND AUTON METHODS

    ArrayList<Double> AwaitingMethodCallingTimes = new ArrayList<Double>();
    ArrayList<String> AwaitingMethodCallingNames = new ArrayList<String>();
    ArrayList<ArrayList<Object>> AwaitingMethodCallingParams = new ArrayList<ArrayList<Object>>();

    private void runMethodAfterSec(String methodName, double delaySeconds, ArrayList<Object> parameters) {
        AwaitingMethodCallingTimes.add(runTime.time() + delaySeconds * 1000);
        AwaitingMethodCallingNames.add(methodName);
        AwaitingMethodCallingParams.add(parameters);
        runAnyPreparedMethods(); // auton better stop being so dumb
    }

    private void runMethodAfterSec(String methodName, double delaySeconds) {
        runMethodAfterSec(methodName, delaySeconds, new ArrayList<Object>());
    }

    private void runMethodAfterSec(String methodName, double delaySeconds, Object Param1) {
        ArrayList<Object> parameters = new ArrayList<Object>();
        parameters.add(Param1);
        runMethodAfterSec(methodName, delaySeconds, parameters);
    }

    private void runMethodAfterSec(String methodName, double delaySeconds, Object Param1, Object Param2) {
        ArrayList<Object> parameters = new ArrayList<Object>();
        parameters.add(Param1);
        parameters.add(Param2);
        runMethodAfterSec(methodName, delaySeconds, parameters);
    }

    private void runAnyPreparedMethods() {
        if (!AwaitingMethodCallingTimes.isEmpty()) { // saves time if the list is empty
            ArrayList<Double> NewAwaitingMethodCallingTimes = new ArrayList<Double>();
            ArrayList<String> NewAwaitingMethodCallingNames = new ArrayList<String>();
            ArrayList<ArrayList<Object>> NewAwaitingMethodCallingParams = new ArrayList<ArrayList<Object>>();

            double arraySize = AwaitingMethodCallingTimes.size();
            for (int i = 0; i < arraySize; i++) {
                telemetry.addLine(AwaitingMethodCallingNames.get(i) + "() running in " + (AwaitingMethodCallingTimes.get(i) - runTime.time()) / 1000 + " sec");

                if (AwaitingMethodCallingTimes.get(i) < runTime.time()) {

                    // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

                    // ArmClaw method names:
                    // openClaw, closeClaw, slowCloseClaw, reallySlowCloseClaw, stopClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, setWristToRaisedFloor, depositSpecimen
                    // dropSamplePickup
                    // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

                    // Parameter methods:
                    // moveArmToPoint, moveClawToFieldCoordinate, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot

                    ArrayList<Object> params = AwaitingMethodCallingParams.get(i);

                    switch (AwaitingMethodCallingNames.get(i)) {
                        case "openClaw": openClaw(); break; // I formatted this this way because it makes it a lot easier to read
                        case "closeClaw": closeClaw(); break;
                        case "slowCloseClaw": slowCloseClaw(); break;
                        case "reallySlowCloseClaw": reallySlowCloseClaw(); break;
                        case "stopClaw": stopClaw(); break;
                        case "setWristToBack": setWristToBack(); break;
                        case "setWristToStraight": setWristToStraight(); break;
                        case "setWristToRaisedFloor": setWristToRaisedFloor(); break;
                        case "setWristToFloorPickup": setWristToFloorPickup(); break;
                        case "dropSamplePickup": dropSamplePickup(); break;
                        case "depositSpecimen": depositSpecimen(); break;
                        case "moveClawToTopBasket": moveClawToTopBasket(); break;
                        case "moveClawToTopRung": moveClawToTopRung(); break;
                        case "moveClawToHumanPickup": moveClawToHumanPickup(); break;
                        case "resetArm": resetArm(); break;
                        case "setWristToAutoAngle": setWristToAutoAngle(); break;
                        case "moveClawToTopRungAuto": moveClawToTopRungAuto(); break;

                        case "moveArmToPoint":
                            moveArmToPoint((Vector2d) params.get(0));
                            break;
                        case "moveClawToFieldCoordinate":
                            moveClawToFieldCoordinate((Vector2d) params.get(0), (double) params.get(1));
                            break;
                        case "holdClawToFieldCoordinate":
                            holdClawAtFieldCoordinate((Vector2d) params.get(0), (double) params.get(1));
                            break;
                        case "moveArmDirectly":
                            moveArmDirectly((double) params.get(0), (double) params.get(1));
                            break;
                        case "setWrist":
                            setWrist((double) params.get(0));
                            break;
                        case "setExtension":
                            ExtensionTargetLength = (double) params.get(0);
                            break;
                        case "setPivot":
                            PivotTargetAngle = (double) params.get(0);
                            break;
                        case "setClaw":
                            setClaw((double) params.get(0));
                            break;
                        case "moveClawToTopRungAdjusted":
                            moveClawToTopRungAdjusted((double) params.get(0), (double) params.get(1));
                            break;
                    }
                } else {
                    NewAwaitingMethodCallingTimes.add(AwaitingMethodCallingTimes.get(i));
                    NewAwaitingMethodCallingNames.add(AwaitingMethodCallingNames.get(i));
                    NewAwaitingMethodCallingParams.add(AwaitingMethodCallingParams.get(i));
                }
            }
            AwaitingMethodCallingTimes = NewAwaitingMethodCallingTimes;
            AwaitingMethodCallingNames = NewAwaitingMethodCallingNames;
            AwaitingMethodCallingParams = NewAwaitingMethodCallingParams;
        }
    }

    // Roadrunner's stupid action things but I simplified them to run only only method after a delay (without interrupting auton)
    public static class RRFinishCommand implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }}


    public class RRRunMethodCommand4 implements Action {
        String methodName;
        double delaySeconds;
        Object Param1;
        Object Param2;
        public RRRunMethodCommand4(String methodName, double delaySeconds, Object Param1, Object Param2) {
            this.methodName = methodName;
            this.delaySeconds = delaySeconds;
            this.Param1 = Param1;
            this.Param2 = Param2;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runMethodAfterSec(methodName, delaySeconds, Param1, Param2);
            return false;
        }
    }

    public class RRRunMethodCommand3 implements Action {
        String methodName;
        double delaySeconds;
        Object Param1;
        public RRRunMethodCommand3(String methodName, double delaySeconds, Object Param1) {
            this.methodName = methodName;
            this.delaySeconds = delaySeconds;
            this.Param1 = Param1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runMethodAfterSec(methodName, delaySeconds, Param1);
            return false;
        }
    }

    public class RRRunMethodCommand2 implements Action {
        String methodName;
        double delaySeconds;
        public RRRunMethodCommand2(String methodName, double delaySeconds) {
            this.methodName = methodName;
            this.delaySeconds = delaySeconds;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runMethodAfterSec(methodName, delaySeconds);
            return false;
        }
    }

    public class RRRunMethodCommand1 implements Action {
        String methodName;
        public RRRunMethodCommand1(String methodName) {
            this.methodName = methodName;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runMethodAfterSec(methodName, 0);
            return false;
        }
    }


    public Action RunMethod(String methodName, double delaySeconds, Object Param1, Object Param2) {
        return new RRRunMethodCommand4(methodName, delaySeconds, Param1, Param2);
    }

    public Action RunMethod(String methodName, double delaySeconds, Object Param1) {
        return new RRRunMethodCommand3(methodName, delaySeconds, Param1);
    }

    public Action RunMethod(String methodName, double delaySeconds) {
        return new RRRunMethodCommand2(methodName, delaySeconds);
    }

    public Action RunMethod(String methodName) {
        return new RRRunMethodCommand1(methodName);
    }

    public class RRWaitCommand implements Action {
        boolean firstStart = true;
        double Delay;
        ElapsedTime waitTimer;
        public RRWaitCommand(double Delay) {
            this.Delay = Delay;
            this.waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstStart) {
                firstStart = false;
                waitTimer.reset();
            }
            return waitTimer.time() < Delay;
        }}
    public Action Wait(double delaySeconds) {
        return new RRWaitCommand(delaySeconds * 1000);
    }


    ElapsedTime timeoutTimer;
    private double TimeoutTime;

    public class RRWaitUntilFinishedAwaiting implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return !AwaitingMethodCallingTimes.isEmpty();
    }}
    public Action waitUntilFinishedAwaiting() {
        return new RRWaitUntilFinishedAwaiting();
    }
    public class RRWaitUntilFinishedAwaitingTimeout implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        updateClawArm();
        return (!AwaitingMethodCallingTimes.isEmpty() || (timeoutTimer.time() > TimeoutTime));
    }}
    public Action waitUntilFinishedAwaiting(double TimeoutSeconds) {
        timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TimeoutTime = TimeoutSeconds * 1000;
        return new RRWaitUntilFinishedAwaitingTimeout();
    }



    public class RRWaitUntilFinishedMoving implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        updateClawArm();
        return (ExtensionPID.closeEnough() && PivotPID.closeEnough());
    }}
    public Action waitUntilFinishedMoving() {
        return new RRWaitUntilFinishedMoving();
    }
    public class RRWaitUntilFinishedMovingTimeout implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        updateClawArm();
        return ((ExtensionPID.closeEnough() && PivotPID.closeEnough()) || (timeoutTimer.time() > TimeoutTime));
    }}
    public Action waitUntilFinishedMoving(double TimeoutSeconds) {
        timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TimeoutTime = TimeoutSeconds * 1000;
        return new RRWaitUntilFinishedMovingTimeout();
    }

}
