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

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
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

    private boolean resetExtensionZero = false;

    ElapsedTime ArmLoopTimer, CommandFrameTime, PIDButtonPressTime, runTime;
    double FrameRate = 1, ArmLoopTime = 0;
    Telemetry telemetry;

    // PID tuning stuff
    boolean PIDButtonPressed = false, PIDIncrementButtonPressed = false;
    double PIDVar = 0, PIDChangeIncrement = 0.01;

    boolean telemetryEnabled = false;

    double CurrentPivotAngleInst = 0, CurrentExtensionLengthInst = 0; // this speeds up the code a lot by only checking sensors one per update
    double ClawAdjustment = 0;

    boolean LoosenClaw = false;

    private DoubleSupplier HeightSupplier, ForwardSupplier;


    public ArmSystem(HardwareMap map, Telemetry inputTelemetry) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        ExtensionF = map.get(DcMotorEx.class, "ExtensionF");
        ExtensionB = map.get(DcMotorEx.class, "ExtensionB");
        ExtensionF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Claw = map.get(Servo.class, "Claw");
        Wrist = map.get(Servo.class, "Wrist");
        ClawTargetPosition = Constants.ClawOpenPosition; // open claw
        WristTargetAngle = 90; // claw can't point straight up at this pivot angle anymore

        CurrentPivotAngleZero = CurrentPivotAngle.getAsDouble();
        CurrentExtensionLengthZero = CurrentExtensionLength.getAsDouble();

        CommandFrameTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // this is how fast the entire code updates / loop time of command scheduler
        FrameRate = 1 / (CommandFrameTime.time() / 1000.0);
        PIDButtonPressTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ArmLoopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // never resets, mainly for auton

        HeightSupplier = SubsystemData.operator::getRightY;
        ForwardSupplier = SubsystemData.operator::getLeftY;

        telemetry = inputTelemetry;

        PivotPID = new PIDController(0.05, 0, 0, 0, Constants.pivotMaxAngle, 0,
                1, 60, 2, true, true,
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.006, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 0, 5, true, false,
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


    public void controlArmTeleOp() {
        GamepadEx gamepad = SubsystemData.operator;
        // Manual arm control when controllers active
        double HeightJoystick = HeightSupplier.getAsDouble();
        double ForwardJoystick = ForwardSupplier.getAsDouble();
        if (functions.inUse(ForwardJoystick) || functions.inUse(HeightJoystick)) {
            backPedalExtension = false;
            if (FrameRate > 2) {
                double ArmThrottle = 0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                if (SubsystemData.operator.getButton(GamepadKeys.Button.LEFT_BUMPER)) { // coords control (one joystick controls height, one controls forward distance)
                    Vector2d targetClawPos = getTargetClawPoint();
                    moveArmToPoint(new Vector2d(targetClawPos.x + Constants.maxManualClawSpeedHorizontal * ForwardJoystick * ArmThrottle / FrameRate, targetClawPos.y + Constants.maxManualClawSpeedVertical * -1 * HeightJoystick * ArmThrottle / FrameRate));
                } else { // direct control (one joystick controls pivot, the other controls extension)
                    double pivotThrottle = 1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength);
                    double manualPivot = PivotTargetAngle + Constants.maxManualPivotSpeed * ForwardJoystick * ArmThrottle * pivotThrottle / FrameRate;
                    double manualExtension = ExtensionTargetLength + Constants.maxManualExtensionSpeed * -1 * HeightJoystick * ArmThrottle / FrameRate;
                    moveArmDirectly(manualPivot, manualExtension);
                }
            }
        }

        LoosenClaw = functions.inUse(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        tunePIDsWithController(SubsystemData.driver);

        updateClawArm();
    }


    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly
        ArmLoopTimer.reset(); // time it takes the arm for 1 update
        double LastArmLoopTime = 0;
        FrameRate = 1 / (CommandFrameTime.time() / 1000.0); // frame rate of the entire code
        CommandFrameTime.reset();
        telemetry.addData("Code FrameRate:", FrameRate);
        telemetry.addData("Arm System Loop Time:", ArmLoopTime);
        telemetry.addData("Drive System Loop Time:", SubsystemData.DrivetrainLoopTime);
        telemetry.addData("HuskyLens Loop Time:", SubsystemData.HuskyLensLoopTime);
        telemetry.addData("HuskyLens Thread Loop Time:", SubsystemData.HuskyLensThreadLoopTime);
        telemetry.addLine(" ");
        // very important that this is updating or it doesn't work correctly, WHY? you may ask, i haven't the foggiest clue
        telemetry.addData("Schrödinger's Encoder:", SubsystemData.brokenDiffyEncoder.getCurrentPosition());


        // Run any delayed auton methods that are ready to run
        runAnyPreparedMethods();


        CurrentPivotAngleInst = CurrentPivotAngle.getAsDouble(); // this speeds up the code a lot by only checking sensors once per update
        CurrentExtensionLengthInst = CurrentExtensionLength.getAsDouble();

        //telemetry.addData("Point 1:", ArmLoopTimer.time() - LastArmLoopTime);
        //LastArmLoopTime = ArmLoopTimer.time();

        // BACKPEDALING

        // The purpose of Backpedal is to retract the extension when the pivot needs to move a lot so that the arm is less likely to hit something and the pivot can rotate faster
        // if backpedal is enabled and the angle already traveled is less than half of the total angle that needs to be traversed

        if (Math.abs(PivotTargetAngle - CurrentPivotAngleInst) > 10) { // backpedals only if the pivot needs to move more than 10 degrees
            backPedalExtension = false; // otherwise disables backpedal
        }
        if (backPedalExtension) {
            if (CurrentExtensionLengthInst < 50) { // if extension is already retracted
                PivotPID.setTarget(PivotTargetAngle); // move pivot
            } // otherwise stop changing the pivot target in the pid (hold the current pivot angle) and retract extension
            ExtensionPID.setTarget(0);
        } else {
            PivotPID.setTarget(PivotTargetAngle);

            // possibly another horizontal expansion limiter
            // if (Math.cos(Math.toRadians(CurrentPivotAngleInst)) * ExtensionTargetLength > Constants.freeHorizontalExpansion) ExtensionTargetLength = ;

            ExtensionPID.setTarget(ExtensionTargetLength);
        }


        // PIVOT

        PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength));
        // set pivot power to pid value + the amount of power needed to counteract gravity at the current pivot angle and current extension length
        double PivotPIDPower = PivotPID.getPower();
        double PivotPower = PivotPIDPower + ((Constants.pivotExtendedGravityPower - Constants.pivotRetractedGravityPower) / Constants.extensionMaxLength * CurrentExtensionLengthInst + Constants.pivotRetractedGravityPower) * Math.cos(Math.toRadians(CurrentPivotAngleInst));
        if (Math.abs(PivotPower) > 0.75) PivotPower = Math.signum(PivotPower) * 0.75; // sets max pivot power TODO: remove this after it has been tested that it works
        if ((CurrentPivotAngleInst < 3 && PivotTargetAngle < 3) || FrameRate < 2) { // stop pivot if it is resting on the mechanical stop or if the framerate is less than 2
            PivotPower = 0;
        }

        Pivot.setPower(PivotPower);



        // EXTENSION

        double ExtensionPIDPower = ExtensionPID.getPower();
        double ExtensionPower = ExtensionPIDPower + Math.sin(Math.toRadians(CurrentPivotAngleInst)) * Constants.extensionGravityPower;
        if (FrameRate < 2) ExtensionPower = 0; // emergency stop extension if framerate is less than 2

        if (CurrentExtensionLengthInst * Math.cos(Math.toRadians(CurrentPivotAngleInst)) > Constants.freeHorizontalExpansion && ExtensionPower > 0) {
            ExtensionPower = 0; // emergency limiter against continuing to power the extension motors outside of the horizontal limit (there are 2 other extension limiters)
        }

        ExtensionF.setPower(ExtensionPower);
        ExtensionB.setPower(ExtensionPower);

        if (resetExtensionZero && ExtensionTargetLength == 0) {
            // if Extension is close to where it thinks 0 is, tell it to keep retracting
            // if (CurrentExtensionLengthInst < 10) Extension.setPower(-0.2);
            // resets 0 if motor can't move and resetting extension zero is enabled
            //if (ExtensionF.getCurrent(CurrentUnit.AMPS) > 6) { // TODO: make sure this isn't a problem when the arm gets temporarily stuck extended or if it is too strong
                //CurrentExtensionLengthZero = CurrentExtensionLengthInst;
                //resetExtensionZero = false;
            //}
        } else resetExtensionZero = false;
        // If the current extension length is ever negative, set the current value to zero
        if (CurrentExtensionLengthInst < 0) CurrentExtensionLengthZero = CurrentExtensionLengthInst;
        if (CurrentExtensionLengthInst > 700) CurrentExtensionLengthZero = CurrentExtensionLengthInst - 700;

        // WRIST AND CLAW

        if (CurrentExtensionLengthInst < 40 && WristTargetAngle - CurrentPivotAngleInst < 45) { // prevents claw servo from hitting slides
            setWrist(45);
        } else if (WristTargetAngle - CurrentPivotAngleInst > 135) { // prevents huskyLens from hitting hook
            setWrist(135);
        } else {
            setWrist(WristTargetAngle - CurrentPivotAngleInst);
        }


        // right trigger slightly loosens the claw's grip
        if (LoosenClaw) ClawAdjustment = 0.05;
        else ClawAdjustment = 0;
        Claw.setPosition(ClawTargetPosition - ClawAdjustment);


        if (!SubsystemData.IMUWorking) telemetry.addLine("IMU HAS STOPPED RESPONDING");
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
            // telemetry.addData("Is backpedaling Arm:", backPedalExtension);
            // telemetry.addData("Wrist Target Angle:", WristTargetAngle);
            // telemetry.addData("Claw Position:", ClawTargetPosition);
            telemetry.addLine(" ");
            telemetry.addData("LT High Current", SubsystemData.DriveMotorHighCurrents[0]);
            telemetry.addData("LB High Current", SubsystemData.DriveMotorHighCurrents[1]);
            telemetry.addData("RT High Current", SubsystemData.DriveMotorHighCurrents[3]);
            telemetry.addData("RB High Current", SubsystemData.DriveMotorHighCurrents[2]);
            telemetry.addLine(" ");
            telemetry.addData("Pivot PID Power:", PivotPIDPower);
            telemetry.addData("Extension PID Power:", ExtensionPIDPower);
            telemetry.addData("Extension Difference", ExtensionTargetLength - CurrentExtensionLengthInst);
            telemetry.addData("Pivot Difference", PivotTargetAngle - CurrentPivotAngleInst);


            telemetry.addLine(" ");
            if (SubsystemData.HuskyLensConnected) telemetry.addLine("HuskyLens Active");
            else telemetry.addLine("HuskyLens not responding");
            HuskyLens.Block[] VisionResults = SubsystemData.Vision;
            telemetry.addData("HuskyLens block count:", VisionResults.length);
            for (HuskyLens.Block value : VisionResults) {
                telemetry.addLine("ID:" + (value.id) + " x:" + (value.x) + " y:" + (value.y) + // Id, center X, center Y
                        " h:" + (value.height) + " w:" + (value.width)); // height, width,  + " ox" + (value.left) + " oy" + (value.top)  origin X, Origin Y
            }
            telemetry.addLine(" \n \n \n \n \n \n \n \n "); // adds spacing so I can actually read the huskylens data without it scrolling
        }

        ArmLoopTime = ArmLoopTimer.time(); // updates how long the arm loop took to run
    }


    private void tunePIDsWithController(GamepadEx inputGamepad) {
        // Allows changing PID variables whilst on the field during testing:
        if (inputGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) PIDChangeIncrement = 0.01;
        else PIDChangeIncrement = 0.0001;

        if (inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !PIDButtonPressed) { // cycle through which PID variable is going to be edited
            PIDVar = PIDVar + 1;
            if (PIDVar > 15) PIDVar = 0;
            PIDButtonPressed = true;
        } else if (inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && !PIDButtonPressed) {
            PIDVar = PIDVar - 1;
            if (PIDVar < 0) PIDVar = 15;
            PIDButtonPressed = true;
        } else if (!inputGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && !inputGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) PIDButtonPressed = false;

        // if the button is held for more than a second, add or subtract constantly
        if ((inputGamepad.getButton(GamepadKeys.Button.DPAD_UP) || inputGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) && !(PIDIncrementButtonPressed && PIDButtonPressTime.time() < 1000)) {
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
            else if (PIDVar == 10) SubsystemData.HeadingTargetPID.kP = functions.round(SubsystemData.HeadingTargetPID.kP + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 11) SubsystemData.HeadingTargetPID.kI = functions.round(SubsystemData.HeadingTargetPID.kI + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 12) SubsystemData.HeadingTargetPID.kD = functions.round(SubsystemData.HeadingTargetPID.kD + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 13) SubsystemData.SwerveModuleKp = functions.round(SubsystemData.SwerveModuleKp + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 14) SubsystemData.SwerveModuleKi = functions.round(SubsystemData.SwerveModuleKi + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 15) SubsystemData.SwerveModuleKd = functions.round(SubsystemData.SwerveModuleKd + PIDChangeIncrement / 10, 5);
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
        else if (PIDVar == 10) telemetry.addData("Editing: Heading Kp - ", SubsystemData.HeadingTargetPID.kP);
        else if (PIDVar == 11) telemetry.addData("Editing: Heading Ki - ", SubsystemData.HeadingTargetPID.kI);
        else if (PIDVar == 12) telemetry.addData("Editing: Heading Kd - ", SubsystemData.HeadingTargetPID.kD);
        else if (PIDVar == 13) telemetry.addData("Editing: Swerve Kp - ", SubsystemData.SwerveModuleKp);
        else if (PIDVar == 14) telemetry.addData("Editing: Swerve Ki - ", SubsystemData.SwerveModuleKi);
        else if (PIDVar == 15) telemetry.addData("Editing: Swerve Kd - ", SubsystemData.SwerveModuleKd);
        // else if (PIDVar == 13) telemetry.addData("Editing: Claw Offset - ", Constants.ClawOpenPosition);
    }


    private Vector2d getCurrentClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(CurrentPivotAngleInst)),
                (CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(CurrentPivotAngleInst)));
    }


    private Vector2d getTargetClawPoint() { // robot centric and based from the pivot axle
        return new Vector2d((ExtensionTargetLength + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(PivotTargetAngle)),
                (ExtensionTargetLength + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(PivotTargetAngle)));
    }


    public Pose2d getCurrentClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d CurrentClawPoint = getCurrentClawPoint(); // Claw Position relative to robot
        return new Pose2d((new Vector2d(
                CurrentClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                CurrentClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public Pose2d getTargetClawPose() { // Note: HEADING IN RADIANS and field centric
        Pose2d CurrentPose = SubsystemData.CurrentRobotPose;
        Vector2d TargetClawPoint = getTargetClawPoint(); // Claw Target Position relative to robot
        return new Pose2d((new Vector2d(
                TargetClawPoint.x * Math.cos(CurrentPose.heading.toDouble()),
                TargetClawPoint.x * Math.sin(CurrentPose.heading.toDouble())) // also uses x because y is the claw height
        ).plus(CurrentPose.position), CurrentPose.heading);
    }


    public double getCurrentClawHeight() { // FROM FIELD TILES instead of from pivot axle
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

        moveArmToPoint(new Vector2d(Math.hypot(DeltaPose.x, DeltaPose.y) - Constants.pivotAxleOffset,
                TargetHeight - Constants.pivotAxleHeight));
    }


    public void moveClawToTopBasket() {
        backPedalExtension = true;
        PivotTargetAngle = 90;
        ExtensionTargetLength = 696;
        WristTargetAngle = 200;
    }


    public void moveClawToTopRung() {
        moveArmToPoint(new Vector2d(Constants.retractedExtensionLength + 100, 660 - Constants.pivotAxleHeight));
        WristTargetAngle = 90;
    }


    public void moveClawToHumanPickup() {
        moveArmToPoint(new Vector2d(Constants.retractedExtensionLength + 100, 28 - Constants.pivotAxleHeight));
        WristTargetAngle = 90;
        openClaw();
    }


    public void moveClawIntoSubmersible() {
        // nothing yet
    }


    public void resetArm() {
        backPedalExtension = true;
        PivotTargetAngle = 0;
        ExtensionTargetLength = 0;
        WristTargetAngle = 180;
        resetExtensionZero = true;
    }




    // WRIST AND CLAW METHODS

    public void setWrist(double Angle) { Wrist.setPosition(1.35 * (Angle / 360) + 0.29); } // make wrist go to that specific angle
    public void openClaw() { ClawTargetPosition = Constants.ClawOpenPosition; }
    public void closeClaw() { ClawTargetPosition = Constants.ClawClosedPosition; }
    public void toggleClaw() {
        if (ClawTargetPosition < (Constants.ClawClosedPosition + Constants.ClawOpenPosition) / 2) openClaw();
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

    private void runAnyPreparedMethods() {
        if (!AwaitingMethodCallingTimes.isEmpty()) { // saves time if the list is empty
            ArrayList<Double> NewAwaitingMethodCallingTimes = new ArrayList<Double>();
            ArrayList<String> NewAwaitingMethodCallingNames = new ArrayList<String>();

            for (int i = 0; i < AwaitingMethodCallingTimes.size(); i++) {
                if (AwaitingMethodCallingTimes.get(i) > runTime.time()) {
                    try {
                        Method method = this.getClass().getMethod(AwaitingMethodCallingNames.get(i));
                        method.invoke(this);
                    } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                } else {
                    NewAwaitingMethodCallingTimes.add(AwaitingMethodCallingTimes.get(i));
                    NewAwaitingMethodCallingNames.add(AwaitingMethodCallingNames.get(i));
                }
            }
            AwaitingMethodCallingTimes = NewAwaitingMethodCallingTimes;
            AwaitingMethodCallingNames = NewAwaitingMethodCallingNames;
        }
    }

    // Roadrunner's stupid action things but I simplified them to run only only method after a delay (without interrupting auton)
    public static class RRFinishCommand implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }}

    public Action RunMethod(String methodName, double delaySeconds) {
        AwaitingMethodCallingTimes.add(runTime.time() + delaySeconds * 1000);
        AwaitingMethodCallingNames.add(methodName);
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

    public class RRWaitUntilFinished implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return !AwaitingMethodCallingTimes.isEmpty();
    }}
    public Action waitUntilFinishedMoving() {
        return new RRWaitUntilFinished();
    }

}
