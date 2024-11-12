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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;


public class ArmSystem extends SubsystemBase {

    private DcMotorEx Pivot, ExtensionF, ExtensionB; // for some reason wants final?
    private final Servo Wrist, Claw;
    private final PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetPosition, PivotTargetAngle, ExtensionTargetLength;

    private double CurrentPivotAngleZero = 0, CurrentExtensionLengthZero = 0;
    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360 - CurrentPivotAngleZero;
    public DoubleSupplier CurrentExtensionLength = () -> ((ExtensionF.getCurrentPosition() / 384.5) * 360 + CurrentPivotAngle.getAsDouble()) / 2088 * 696 - CurrentExtensionLengthZero;
    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving

    ElapsedTime ArmLoopTimer, CommandFrameTime, PIDButtonPressTime, runTime, resetArmAlignmentHoldTimer;
    double FrameRate = 1, ArmLoopTime = 0;
    Telemetry telemetry;
    boolean telemetryEnabled = false;

    // PID tuning stuff
    boolean PIDButtonPressed = false, PIDIncrementButtonPressed = false;
    int PIDVar = 0;
    double PIDChangeIncrement = 0.01;


    double CurrentPivotAngleInst = 0, CurrentExtensionLengthInst = 0; // this speeds up the code a lot by only checking sensors one per update

    double ClawAdjustment = 0;
    private boolean LoosenClaw = false, cameraToggle, activeExtensionReset = false;

    private int CurrentlyReadyPreset = 0; // allows pressing a preset button twice to complete the second part of its action

    // Vector2d FieldCoordHoldPos;
    // double FieldCoordHoldHeight;

    HuskyLensCamera camera;



    public ArmSystem(HardwareMap map, Telemetry inputTelemetry) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        ExtensionF = map.get(DcMotorEx.class, "ExtensionF");
        ExtensionB = map.get(DcMotorEx.class, "ExtensionB");
        ExtensionF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
        ClawTargetPosition = Constants.ClawOpenPosition; // open claw
        WristTargetAngle = 90; // claw can't point straight up when arm is down anymore because of the hook

        SubsystemData.HoldClawFieldPos = false; // makes sure this is off on startup

        CurrentPivotAngleZero = CurrentPivotAngle.getAsDouble();
        CurrentExtensionLengthZero = CurrentExtensionLength.getAsDouble();

        camera = new HuskyLensCamera(map);


        // Timers
        CommandFrameTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // this is how fast the entire code updates / loop time of command scheduler
        FrameRate = 1 / (CommandFrameTime.time() / 1000.0);
        PIDButtonPressTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ArmLoopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // never resets, mainly for auton
        resetArmAlignmentHoldTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // resets extension if resetArm is held

        telemetry = inputTelemetry;

        PivotPID = new PIDController(0.05, 0, 0, 0, Constants.pivotMaxAngle, 0,
                1, 0, 135, 3, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.006, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 0, 0, 5, true, false,
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


    public void controlArmTeleOp() {
        GamepadEx gamepad = SubsystemData.operator;
        // Manual arm control when controllers active
        if (functions.inUse(gamepad.getRightY()) || functions.inUse(gamepad.getLeftY())) {
            backPedalExtension = false;
            if (FrameRate > 2) {
                double ArmThrottle = 0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                if (SubsystemData.operator.getButton(GamepadKeys.Button.LEFT_BUMPER)) { // coords control (one joystick controls height, the other controls forward and strafe)
                    Vector2d targetClawPos = getTargetClawPoint();
                    moveArmToPoint(new Vector2d(
                            targetClawPos.x + Constants.maxManualClawSpeedHorizontal * gamepad.getLeftY() * ArmThrottle / FrameRate,
                            targetClawPos.y + Constants.maxManualClawSpeedVertical * -1 * gamepad.getRightY() * ArmThrottle / FrameRate));

                    //Pose2d targetClawFieldCoord = getTargetClawPose();
                    //FieldCoordHoldPos = new Vector2d((targetClawFieldCoord.position.x * 25.4 + Constants.maxManualClawSpeedHorizontal * -1 * gamepad.getLeftY() * ArmThrottle / FrameRate) / 25.4, (targetClawFieldCoord.position.y * 25.4 + Constants.maxManualClawSpeedHorizontal * -1 * gamepad.getLeftX() * ArmThrottle / FrameRate) / 25.4);
                    //FieldCoordHoldHeight = getTargetClawHeight() + Constants.maxManualClawSpeedVertical * gamepad.getRightY() * ArmThrottle / FrameRate;
                    //SubsystemData.HoldClawFieldPos = true;
                } else { // direct control (one joystick controls pivot, the other controls extension)
                    double pivotThrottle = 1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength);
                    moveArmDirectly(
                            PivotTargetAngle + Constants.maxManualPivotSpeed * -1 * gamepad.getRightY() * ArmThrottle * pivotThrottle / FrameRate,
                            ExtensionTargetLength + Constants.maxManualExtensionSpeed * gamepad.getLeftY() * ArmThrottle / FrameRate);
                    //SubsystemData.HoldClawFieldPos = false;
                }
            }
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && resetArmAlignmentHoldTimer.time() > 2000) {
            activeExtensionReset = true; // keep retracting extension past limits until button is unpressed and then reset extension length
        } else if (!gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            resetArmAlignmentHoldTimer.reset();
            activeExtensionReset = false;
        }


        // AUTO AIM
        double AutoAimTrigger = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (functions.inUse(AutoAimTrigger)) {
            SubsystemData.OperatorTurningPower = 0; // make sure not actively turning
            if (!cameraToggle) {
                cameraToggle = true;
                camera.StartHuskyLensThread();
            }
            camera.ScanForSample();

            if (SubsystemData.CameraSeesValidObject) {

                // needs to move differently based on the wrist current angle
                Vector2d targetClawPosition = getTargetClawPoint();
                if (WristTargetAngle > 45) {
                    moveArmToPoint(new Vector2d(
                            targetClawPosition.x,
                            targetClawPosition.y + Constants.maxCameraTargetingSpeed * AutoAimTrigger * -1 * (SubsystemData.CameraTargetPixelsY / 120) / FrameRate));
                } else {
                    moveArmToPoint(new Vector2d(
                            targetClawPosition.x + Constants.maxCameraTargetingSpeed * AutoAimTrigger * (SubsystemData.CameraTargetPixelsY / 120) / FrameRate,
                            targetClawPosition.y));
                }

                // moveArmDirectly(PivotTargetAngle, ExtensionTargetLength + Constants.maxCameraTargetingSpeed * AutoAimTrigger * (SubsystemData.CameraTargetPixelsY / 120) / FrameRate);

                // slows down camera auto aim turn speed the further extension is extended
                double CameraTargetingTurnThrottle = (Constants.pivotAxleOffset + Constants.retractedExtensionLength) / (Constants.pivotAxleOffset + Constants.retractedExtensionLength + CurrentExtensionLengthInst);
                //SubsystemData.OperatorTurningPower = -0.15 * AutoAimTrigger * CameraTargetingTurnThrottle * (SubsystemData.CameraTargetPixelsX / 160);

                double AutoAimHeadingChange = Constants.maxCameraTargetingTurnSpeed * AutoAimTrigger * CameraTargetingTurnThrottle * (SubsystemData.CameraTargetPixelsX / 160);
                SubsystemData.AutoAimHeading = AutoAimHeadingChange / FrameRate;
                SubsystemData.OverrideDrivetrainRotation = true;
                telemetry.addData("Auto Aim Heading:", SubsystemData.AutoAimHeading);
                telemetry.addData("Auto Aim Change:", AutoAimHeadingChange);

            } else SubsystemData.OverrideDrivetrainRotation = false;

        } else if (cameraToggle) { // turn off huskylens thread when not in use cause its laggy
            cameraToggle = false;
            camera.EndHuskyLensThread();

            // operator can control turn when not auto aiming and driver isn't turning
        } else {
            SubsystemData.OverrideDrivetrainRotation = false;
        }

        SubsystemData.OperatorTurningPower = -0.2 * Math.pow(gamepad.getLeftX(), 3);

        tunePIDsWithController(SubsystemData.driver);

        updateClawArm();
    }


    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly
        ArmLoopTimer.reset(); // time it takes the arm for 1 update
        FrameRate = 1 / (CommandFrameTime.time() / 1000.0); // frame rate of the entire code
        CommandFrameTime.reset();
        if (!SubsystemData.IMUWorking) telemetry.addLine("IMU HAS STOPPED RESPONDING");
        telemetry.addData("Code FrameRate:", FrameRate);
        telemetry.addData("Arm System Loop Time:", ArmLoopTime);
        telemetry.addData("Drive System Loop Time:", SubsystemData.DrivetrainLoopTime);
        telemetry.addData("HuskyLens Loop Time:", SubsystemData.HuskyLensLoopTime);
        telemetry.addData("HuskyLens Thread Loop Time:", SubsystemData.HuskyLensThreadLoopTime);
        telemetry.addLine(" ");
        // very important that this is updating or it doesn't work correctly, WHY? you may ask, idk
        telemetry.addData("Schrödinger's Encoder:", SubsystemData.brokenDiffyEncoder.getCurrentPosition());


        CurrentPivotAngleInst = CurrentPivotAngle.getAsDouble(); // this speeds up the code a lot by only checking sensors once per update
        CurrentExtensionLengthInst = CurrentExtensionLength.getAsDouble();


        // Run any delayed auton methods that are ready to run
        runAnyPreparedMethods();


        //telemetry.addData("Point 1:", ArmLoopTimer.time() - LastArmLoopTime);
        //LastArmLoopTime = ArmLoopTimer.time();


        // HOLD CLAW AT A FIELD COORD
        /*
        if (SubsystemData.HoldClawFieldPos) {
            Vector2d DistanceVector = FieldCoordHoldPos.minus(SubsystemData.CurrentRobotPose.position);
            if (!(Math.hypot(DistanceVector.x, DistanceVector.y) > Constants.extensionMaxLength)) {
                moveClawToFieldCoordinate(FieldCoordHoldPos, FieldCoordHoldHeight);
            } else SubsystemData.HoldClawFieldPos = false;
        }
         */


        // Keep targets within limits
        if (PivotTargetAngle > Constants.pivotMaxAngle) PivotTargetAngle = Constants.pivotMaxAngle;
        else if (PivotTargetAngle < 0) PivotTargetAngle = 0;
        if (ExtensionTargetLength > Constants.extensionMaxLength + 2) ExtensionTargetLength = Constants.extensionMaxLength + 2;
        else if (ExtensionTargetLength < -2) ExtensionTargetLength = -2;



        // BACKPEDALING

        // The purpose of Backpedal is to retract the extension when the pivot needs to move a lot so that the arm is less likely to hit something and the pivot can rotate faster
        // if backpedal is enabled and the angle already traveled is less than half of the total angle that needs to be traversed

        // backpedals only if the pivot needs to move more than 10 degrees
        if (Math.abs(PivotTargetAngle - CurrentPivotAngleInst) < 10) backPedalExtension = false;

        if (backPedalExtension) {
            if (CurrentPivotAngleInst < 10) { // if claw could be hanging low to the ground
                PivotPID.setTarget(15); // raise pivot a little first
            } else if (CurrentExtensionLengthInst < 70) { // if extension is already retracted
                PivotPID.setTarget(PivotTargetAngle); // move pivot
                ExtensionPID.setTarget(0);
            } else ExtensionPID.setTarget(0); // otherwise stop changing the pivot target in the pid (hold the current pivot angle) and retract extension
        } else {
            PivotPID.setTarget(PivotTargetAngle);

            // possibly another horizontal extension limiter
            // if (Math.cos(Math.toRadians(CurrentPivotAngleInst)) * ExtensionTargetLength > Constants.freeHorizontalExpansion) ExtensionTargetLength = ;

            ExtensionPID.setTarget(ExtensionTargetLength);
        }


        // PIVOT

        PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength));
        // set pivot power to pid value + the amount of power needed to counteract gravity at the current pivot angle and current extension length
        double PivotPIDPower = PivotPID.getPower();
        double PivotPower = PivotPIDPower + ((Constants.pivotExtendedGravityPower - Constants.pivotRetractedGravityPower) / Constants.extensionMaxLength * CurrentExtensionLengthInst + Constants.pivotRetractedGravityPower) * Math.cos(Math.toRadians(CurrentPivotAngleInst));
        // if (Math.abs(PivotPower) > 0.75) PivotPower = Math.signum(PivotPower) * 0.75; // sets max pivot power
        if ((CurrentPivotAngleInst < 3 && PivotTargetAngle < 3) || FrameRate < 2) { // stop pivot if it is resting on the mechanical stop or if the framerate is less than 2
            PivotPower = 0;
        }

        Pivot.setPower(PivotPower);



        // EXTENSION

        double ExtensionPIDPower = ExtensionPID.getPower();
        double ExtensionPower = ExtensionPIDPower + Math.sin(Math.toRadians(CurrentPivotAngleInst)) * Constants.extensionGravityPower;
        if (FrameRate < 2) ExtensionPower = 0; // emergency stop extension if framerate is less than 2

        if (CurrentExtensionLengthInst * Math.cos(Math.toRadians(CurrentPivotAngleInst)) > Constants.freeHorizontalExpansion - 50 && ExtensionPower > 0) {
            ExtensionPower = 0; // limiter against continuing to power the extension motors outside of the horizontal limit (there are 3 other extension limiters)
        }

        if (activeExtensionReset) { // reset extension length
            ExtensionF.setPower(-0.4);
            ExtensionB.setPower(-0.4);
        } else {
            ExtensionF.setPower(ExtensionPower);
            ExtensionB.setPower(ExtensionPower);
        }


        // If the current extension length is ever outside the limits, move the zero so it is again
        if (CurrentExtensionLengthInst < -1) CurrentExtensionLengthZero = CurrentExtensionLengthInst - (-1);
        if (CurrentExtensionLengthInst > 697) CurrentExtensionLengthZero = CurrentExtensionLengthInst - 697;

        // if (RealignExtension && ExtensionF.getCurrent(CurrentUnit.AMPS) > 2) {
            // CurrentExtensionLengthZero = 0; // TODO: this may break some stuff
            // RealignExtension = false;
        // }

        // WRIST AND CLAW

        if (CurrentExtensionLengthInst < 40 && WristTargetAngle - CurrentPivotAngleInst < 45) { // prevents claw servo from hitting slides
            setWrist(45);
        } else if (WristTargetAngle - CurrentPivotAngleInst > 135) { // prevents huskyLens from hitting hook
            setWrist(135);
        } else {
            setWrist(WristTargetAngle - CurrentPivotAngleInst);
        }


        // slightly loosens the claw's grip
        if (LoosenClaw) ClawAdjustment = 0.05;
        else ClawAdjustment = 0;
        Claw.setPosition(ClawTargetPosition - ClawAdjustment);


        telemetry.addData("Heading:", Math.toDegrees(SubsystemData.CurrentRobotPose.heading.toDouble()));
        telemetry.addLine("Robot Pose (in) X: " +
                functions.round(SubsystemData.CurrentRobotPose.position.x, 2) + " Y: " +
                functions.round(SubsystemData.CurrentRobotPose.position.y, 2));

        if (telemetryEnabled) { // a lot of telemetry slows the code down so I made it toggleable
            telemetry.addLine(" ");
            Pose2d CurrentClawPosition = getCurrentClawPose(); // avoiding calling this method twice
            telemetry.addLine("Claw Field Pose X: " +
                    functions.round(CurrentClawPosition.position.x, 2) + " Y: " +
                    functions.round(CurrentClawPosition.position.y, 2) + " H: " +
                    functions.round(getCurrentClawHeight(), 2));
            //Pose2d TargetClawPosition = getTargetClawPose(); // avoiding calling this method twice
            //telemetry.addLine("Target Claw Pose X: " + functions.round(TargetClawPosition.position.x, 2) + " Y: " + functions.round(TargetClawPosition.position.y, 2) + " H: " + functions.round(getTargetClawHeight(), 2));
            telemetry.addData("Pivot Motor Power:", PivotPower);
            telemetry.addData("Extension Motor Power:", ExtensionPower);
            telemetry.addData("Extension Motor Current:", ExtensionF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Extension Target Length:", ExtensionTargetLength);
            telemetry.addData("Extension Current Length:", CurrentExtensionLengthInst);
            telemetry.addData("Pivot Target Angle:", PivotTargetAngle);
            telemetry.addData("Pivot Current Angle:", CurrentPivotAngleInst);
            // telemetry.addData("Wrist Target Angle:", WristTargetAngle);
            // telemetry.addData("Claw Position:", ClawTargetPosition);
            //telemetry.addLine(" ");
            //telemetry.addData("LT High Current", SubsystemData.DriveMotorHighCurrents[0]);
            //telemetry.addData("LB High Current", SubsystemData.DriveMotorHighCurrents[1]);
            //telemetry.addData("RT High Current", SubsystemData.DriveMotorHighCurrents[3]);
            //telemetry.addData("RB High Current", SubsystemData.DriveMotorHighCurrents[2]);
            telemetry.addLine(" ");
            telemetry.addData("Pivot PID Power:", PivotPIDPower);
            telemetry.addData("Extension PID Power:", ExtensionPIDPower);
            telemetry.addData("Extension Difference", ExtensionTargetLength - CurrentExtensionLengthInst);
            telemetry.addData("Pivot Difference", PivotTargetAngle - CurrentPivotAngleInst);


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
                telemetry.addLine(" ");
            }
            HuskyLens.Block[] VisionResults = SubsystemData.Vision;
            telemetry.addData("HuskyLens block count:", VisionResults.length);
            for (HuskyLens.Block value : VisionResults) {
                telemetry.addLine("ID:" + (value.id) + " x:" + (value.x) + " y:" + (value.y) + // Id, center X, center Y
                        " h:" + (value.height) + " w:" + (value.width)); // height, width,  + " ox" + (value.left) + " oy" + (value.top)  origin X, Origin Y
            }
            telemetry.addLine(" \n \n \n \n \n \n \n "); // adds spacing so I can actually read the huskylens data without it scrolling
        }

        ArmLoopTime = ArmLoopTimer.time(); // updates how long the arm loop took to run
    }


    private void tunePIDsWithController(GamepadEx inputGamepad) {
        // Allows changing PID variables whilst on the field during testing:
        if (inputGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) PIDChangeIncrement = 0.01;
        else PIDChangeIncrement = 0.0001;

        if (inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !PIDButtonPressed) { // cycle through which PID variable is going to be edited
            PIDVar = PIDVar + 1;
            if (PIDVar > 17) PIDVar = 0;
            PIDButtonPressed = true;
        } else if (inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && !PIDButtonPressed) {
            PIDVar = PIDVar - 1;
            if (PIDVar < 0) PIDVar = 17;
            PIDButtonPressed = true;
        } else if (!inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) PIDButtonPressed = false;

        // if the button is held for more than a second, add or subtract constantly
        if ((inputGamepad.getButton(GamepadKeys.Button.DPAD_UP) || inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) && !(PIDIncrementButtonPressed && PIDButtonPressTime.time() < 1000)) {
            if (inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) PIDChangeIncrement = -PIDChangeIncrement; // subtract if down

            switch (PIDVar) {
                case 1: ExtensionPID.kP = functions.round(ExtensionPID.kP + PIDChangeIncrement, 4); break;
                case 2: ExtensionPID.kI = functions.round(ExtensionPID.kI + PIDChangeIncrement, 4); break;
                case 3: ExtensionPID.kD = functions.round(ExtensionPID.kD + PIDChangeIncrement, 4); break;
                case 4: Constants.extensionGravityPower = functions.round(Constants.extensionGravityPower + PIDChangeIncrement * 10, 3); break;
                case 5: PivotPID.kP = functions.round(PivotPID.kP + PIDChangeIncrement, 4); break;
                case 6: PivotPID.kI = functions.round(PivotPID.kI + PIDChangeIncrement, 4); break;
                case 7: PivotPID.kD = functions.round(PivotPID.kD + PIDChangeIncrement, 4); break;
                case 8: Constants.pivotRetractedGravityPower = functions.round(Constants.pivotRetractedGravityPower + PIDChangeIncrement * 10, 3); break;
                case 9: Constants.pivotExtendedGravityPower = functions.round(Constants.pivotExtendedGravityPower + PIDChangeIncrement * 10, 3); break;
                case 10: SubsystemData.HeadingTargetPID.kP = functions.round(SubsystemData.HeadingTargetPID.kP + PIDChangeIncrement / 10, 5); break;
                case 11: SubsystemData.HeadingTargetPID.kI = functions.round(SubsystemData.HeadingTargetPID.kI + PIDChangeIncrement / 10, 5); break;
                case 12: SubsystemData.HeadingTargetPID.kD = functions.round(SubsystemData.HeadingTargetPID.kD + PIDChangeIncrement / 10, 5); break;
                case 13: SubsystemData.HeadingTargetPID.initialPower = functions.round(SubsystemData.HeadingTargetPID.initialPower + PIDChangeIncrement, 4); break;
                case 14: SubsystemData.HeadingTargetPID.minPower = functions.round(SubsystemData.HeadingTargetPID.minPower + PIDChangeIncrement, 4); break;
                case 15: SubsystemData.AutonGain = functions.round(SubsystemData.AutonGain + PIDChangeIncrement, 4); break;
                case 16: SubsystemData.AutonMinPower = functions.round(SubsystemData.AutonMinPower + PIDChangeIncrement, 4); break;
                case 17: SubsystemData.SwerveModuleDriveSharpness = SubsystemData.SwerveModuleDriveSharpness + (int) Math.round(Math.signum(PIDChangeIncrement)); break;
            }
            if (!PIDIncrementButtonPressed) { // only happens once when the button is first pressed
                PIDButtonPressTime.reset(); // set the time that the button started being pressed to 0
                PIDIncrementButtonPressed = true;
            }
        } else if (!inputGamepad.getButton(GamepadKeys.Button.DPAD_UP) && !inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            PIDIncrementButtonPressed = false;
        }
        switch (PIDVar) {
            case 0: telemetry.addLine("Not Editing PIDs"); break;
            case 1: telemetry.addData("Editing: Extension Kp -", ExtensionPID.kP); break;
            case 2: telemetry.addData("Editing: Extension Ki -", ExtensionPID.kI); break;
            case 3: telemetry.addData("Editing: Extension Kd -", ExtensionPID.kD); break;
            case 4: telemetry.addData("Editing: Extension Gravity -", Constants.extensionGravityPower); break;
            case 5: telemetry.addData("Editing: Pivot Kp -", PivotPID.kP); break;
            case 6: telemetry.addData("Editing: Pivot Ki -", PivotPID.kI); break;
            case 7: telemetry.addData("Editing: Pivot Kd -", PivotPID.kD); break;
            case 8: telemetry.addData("Editing: Pivot Retracted Gravity -", Constants.pivotRetractedGravityPower); break;
            case 9: telemetry.addData("Editing: Pivot Extended Gravity -", Constants.pivotExtendedGravityPower); break;
            case 10: telemetry.addLine("Editing: Heading Kp - " + SubsystemData.HeadingTargetPID.kP); break; // addData only prints doubles up to 4 decimal places
            case 11: telemetry.addLine("Editing: Heading Ki - " + SubsystemData.HeadingTargetPID.kI); break;
            case 12: telemetry.addLine("Editing: Heading Kd - " + SubsystemData.HeadingTargetPID.kD); break;
            case 13: telemetry.addData("Editing: Heading initialPower - ", SubsystemData.HeadingTargetPID.initialPower); break;
            case 14: telemetry.addData("Editing: Heading minPower - ", SubsystemData.HeadingTargetPID.minPower); break;
            case 15: telemetry.addData("Editing: Auton Gain - ", SubsystemData.AutonGain); break;
            case 16: telemetry.addData("Editing: Auton MinPower - ", SubsystemData.AutonMinPower); break;
            case 17: telemetry.addData("Editing: Swerve Module Sharpness - ", SubsystemData.SwerveModuleDriveSharpness); break;
        }
    }


    private Vector2d getCurrentClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(CurrentPivotAngleInst)),
                (CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(CurrentPivotAngleInst)));
    }


    private Vector2d getTargetClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((ExtensionTargetLength + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(PivotTargetAngle)),
                (ExtensionTargetLength + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(PivotTargetAngle)));
    }


    public Pose2d getCurrentClawPose() { // Note: HEADING IN RADIANS and field centric in inches
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d CurrentClawPoint = getCurrentClawPoint(); // Claw Position relative to robot
        return new Pose2d((new Vector2d(
                CurrentClawPoint.x / 25.4 * Math.cos(CurrentPose.heading.toDouble()),
                CurrentClawPoint.x / 25.4 * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public Pose2d getTargetClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d TargetClawPoint = getTargetClawPoint(); // Claw Target Position relative to robot
        return new Pose2d((new Vector2d(
                TargetClawPoint.x / 25.4 * Math.cos(CurrentPose.heading.toDouble()),
                TargetClawPoint.x / 25.4 * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
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
        if (PivotTargetAngle < 0) PivotTargetAngle = 0; // prevent pivot from exceeding max positions
        else if (PivotTargetAngle > Constants.pivotMaxAngle) PivotTargetAngle = Constants.pivotMaxAngle;
        // extension get realigned because it skips a lot instead of locking it at max positions
        // if (ExtensionTargetLength < 0) ExtensionTargetLength = 0;
        // else if (ExtensionTargetLength > Constants.extensionMaxLength) ExtensionTargetLength = Constants.extensionMaxLength;
    }


    private void moveArmToPoint(Vector2d point) { // units:mm, makes pivot and extension work together to go to a set point relative to the pivot axle on the robot
        double Y = point.y;
        double X = point.x;

        // This horizontal extension limiter allows for the arm to at least go to the correct height if the target x is outside the expansion limit
        if (X > Constants.freeHorizontalExpansion + Constants.retractedExtensionLength) X = Constants.freeHorizontalExpansion + Constants.retractedExtensionLength;
        moveArmDirectly(Math.toDegrees(Math.atan2(Y, X)), Math.hypot(X, Y) - Constants.retractedExtensionLength);
    }


    public void moveClawToFieldCoordinate(Vector2d TargetClawPos, double TargetHeight) { // needs to be called constantly while in use, HEIGHT IS FROM FIELD TILES
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d DeltaPose = TargetClawPos.minus(CurrentPose.position); // Vector of where the arm needs to go relative to the robot

        // tell the drivetrain to point towards the target point
        SubsystemData.OverrideDrivetrainTargetHeading = Math.toDegrees(Math.atan2(DeltaPose.y, DeltaPose.x));
        SubsystemData.OverrideDrivetrainRotation = true;

        moveArmToPoint(new Vector2d(Math.hypot(DeltaPose.x * 25.4, DeltaPose.y * 25.4) - Constants.pivotAxleOffset,
                TargetHeight - Constants.pivotAxleHeight));
    }


    // CurrentlyReadyPreset:
    // 0 = none
    // 1 = moveClawToTopBasket
    // 2 = moveClawToTopRung
    // 3 = moveClawToHumanPickup
    // 4 = moveClawIntoSubmersible


    public void resetArm() {
        CurrentlyReadyPreset = 0;
        SubsystemData.HoldClawFieldPos = false;
        backPedalExtension = true;
        PivotTargetAngle = 0;
        ExtensionTargetLength = 0;
        WristTargetAngle = 180;
    }


    public void moveClawToTopBasket() {
        if (CurrentlyReadyPreset == 1) {
            enableLoosenClaw();
            runMethodAfterSec("openClaw", 0.5);
            runMethodAfterSec("disableLoosenClaw", 0.5);
            CurrentlyReadyPreset = 0;
        } else {
            backPedalExtension = true;
            PivotTargetAngle = 90;
            ExtensionTargetLength = 696;
            WristTargetAngle = 200;
            CurrentlyReadyPreset = 1;
        }
    }


    public void moveClawToTopRung() {
        if (CurrentlyReadyPreset == 2) { // second action
            WristTargetAngle = 50;
            depositSpecimen();
            CurrentlyReadyPreset = 0;
        } else { // normal action
            backPedalExtension = true;
            moveArmToPoint(new Vector2d(Constants.retractedExtensionLength + 75, 760 - Constants.pivotAxleHeight));
            WristTargetAngle = 50;
            CurrentlyReadyPreset = 2;
        }
    }


    public void depositSpecimen() { // onto a rung
        ExtensionTargetLength = CurrentExtensionLengthInst - 150;
        runMethodAfterSec("openClaw", 1.5); // TODO: might need to remove this
    }


    public void driveClampSpecimenPosition() { // goes to the position needed to just drive into the rungs and instantly clamp the specimen
        backPedalExtension = true;
        moveArmToPoint(new Vector2d(Constants.retractedExtensionLength + 75, 700 - Constants.pivotAxleHeight));
        WristTargetAngle = 180;
    }


    public void moveClawToHumanPickup() {
        if (CurrentlyReadyPreset == 3) { // second action
            Vector2d currentArmPoint = getTargetClawPoint();
            closeClaw();
            runMethodAfterSec("moveArmToPoint", 1.5, new Vector2d(currentArmPoint.x, currentArmPoint.y + 120));
            CurrentlyReadyPreset = 0;
        } else {
            backPedalExtension = true;
            moveArmToPoint(new Vector2d(Constants.retractedExtensionLength + 100, 280 - Constants.pivotAxleHeight));
            WristTargetAngle = 90;
            openClaw();
            CurrentlyReadyPreset = 3;
        }
    }


    public void moveClawIntoSubmersible() {
        CurrentlyReadyPreset = 4;
        backPedalExtension = true;
        //SubsystemData.HoldClawFieldPos = true;
        moveClawToFieldCoordinate(new Vector2d(0, 0), 120);
        setWristToFloorPickup();
        openClaw();
    }





    // WRIST AND CLAW METHODS

    public void setWrist(double Angle) { Wrist.setPosition(1.35 * (Angle / 360) + 0.29); } // make wrist go to that specific angle
    public void openClaw() { ClawTargetPosition = Constants.ClawOpenPosition; }
    public void closeClaw() { ClawTargetPosition = Constants.ClawClosedPosition; }
    public void toggleClaw() {
        if (ClawTargetPosition > (Constants.ClawClosedPosition + Constants.ClawOpenPosition) / 2) openClaw();
        else closeClaw();
    }
    public void enableLoosenClaw() { LoosenClaw = true; }
    public void disableLoosenClaw() { LoosenClaw = false; }
    public void toggleLoosenClaw() { LoosenClaw = !LoosenClaw; }
    public void setWristToCenter() { WristTargetAngle = 90; }
    public void setWristToBasket() { WristTargetAngle = 190; }
    public void setWristToFloorPickup() { WristTargetAngle = 0; }


    public void toggleTelemetry() { telemetryEnabled = !telemetryEnabled; }



    // ROADRUNNER AND AUTON METHODS

    ArrayList<Double> AwaitingMethodCallingTimes = new ArrayList<Double>();
    ArrayList<String> AwaitingMethodCallingNames = new ArrayList<String>();
    ArrayList<ArrayList<Object>> AwaitingMethodCallingParams = new ArrayList<ArrayList<Object>>();

    private void runMethodAfterSec(String methodName, double delaySeconds, ArrayList<Object> parameters) {
        AwaitingMethodCallingTimes.add(runTime.time() + delaySeconds * 1000);
        AwaitingMethodCallingNames.add(methodName);
        AwaitingMethodCallingParams.add(parameters);
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

                if (AwaitingMethodCallingTimes.get(i) > runTime.time()) {

                    // ArmClaw method names:
                    // openClaw, closeClaw, toggleClaw, setWristToCenter, setWristToBasket, setWristToFloorPickup, depositSpecimen
                    // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw,
                    // moveClawToTopBasket, moveClawToTopRung, moveClawToHumanPickup, resetArm,

                    // Parameter methods:
                    // moveArmToPoint, moveClawToFieldCoordinate, moveArmDirectly, setWrist

                    ArrayList<Object> params = AwaitingMethodCallingParams.get(i);

                    switch (AwaitingMethodCallingNames.get(i)) {
                        case "openClaw": openClaw(); break; // I formatted this this way because it looks a lot easier to read
                        case "closeClaw": closeClaw(); break;
                        case "toggleClaw": toggleClaw(); break;
                        case "setWristToCenter": setWristToCenter(); break;
                        case "setWristToBasket": setWristToBasket(); break;
                        case "setWristToFloorPickup": setWristToFloorPickup(); break;
                        case "depositSpecimen": depositSpecimen(); break;
                        case "enableLoosenClaw": enableLoosenClaw(); break;
                        case "disableLoosenClaw": disableLoosenClaw(); break;
                        case "toggleLoosenClaw": toggleLoosenClaw(); break;
                        case "moveClawToTopBasket": moveClawToTopBasket(); break;
                        case "moveClawToTopRung": moveClawToTopRung(); break;
                        case "moveClawToHumanPickup": moveClawToHumanPickup(); break;
                        case "resetArm": resetArm(); break;

                        case "moveArmToPoint":
                            moveArmToPoint((Vector2d) params.get(0));
                            break;
                        case "moveClawToFieldCoordinate":
                            moveClawToFieldCoordinate((Vector2d) params.get(0), (double) params.get(1));
                            break;
                        case "moveArmDirectly":
                            moveArmDirectly((double) params.get(0), (double) params.get(1));
                            break;
                        case "setWrist":
                            setWrist((double) params.get(0));
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


    public Action RunMethod(String methodName, double delaySeconds, Object Param1, Object Param2) {
        runMethodAfterSec(methodName, delaySeconds, Param1, Param2);
        return new RRFinishCommand();
    }

    public Action RunMethod(String methodName, double delaySeconds, Object Param1) {
        runMethodAfterSec(methodName, delaySeconds, Param1);
        return new RRFinishCommand();
    }

    public Action RunMethod(String methodName, double delaySeconds) {
        runMethodAfterSec(methodName, delaySeconds);
        return new RRFinishCommand();
    }

    public Action RunMethod(String methodName) {
        return RunMethod(methodName, 0);
    }

    double StartWaitTime, WaitTime;
    public class RRWaitCommand implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return runTime.time() - StartWaitTime < WaitTime;
    }}
    public Action Wait(double delaySeconds) {
        StartWaitTime = runTime.time();
        WaitTime = delaySeconds * 1000;
        return new RRWaitCommand();
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
        return (!AwaitingMethodCallingTimes.isEmpty() || (timeoutTimer.time() > TimeoutTime));
    }}
    public Action waitUntilFinishedAwaiting(double TimeoutSeconds) {
        timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TimeoutTime = TimeoutSeconds * 1000;
        return new RRWaitUntilFinishedAwaitingTimeout();
    }



    public class RRWaitUntilFinishedMoving implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return (ExtensionPID.closeEnough() && PivotPID.closeEnough());
    }}
    public Action waitUntilFinishedMoving() {
        return new RRWaitUntilFinishedMoving();
    }
    public class RRWaitUntilFinishedMovingTimeout implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return ((ExtensionPID.closeEnough() && PivotPID.closeEnough()) || (timeoutTimer.time() > TimeoutTime));
    }}
    public Action waitUntilFinishedMoving(double TimeoutSeconds) {
        timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TimeoutTime = TimeoutSeconds * 1000;
        return new RRWaitUntilFinishedMovingTimeout();
    }

}
