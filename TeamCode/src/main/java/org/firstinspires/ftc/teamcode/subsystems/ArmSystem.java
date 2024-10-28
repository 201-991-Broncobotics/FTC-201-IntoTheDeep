package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
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

    private DcMotorEx Pivot, ExtensionF, ExtensionB; // for some reason wants final?
    private final Servo Wrist, Claw;
    private final PIDController PivotPID, ExtensionPID;


    private double WristTargetAngle, ClawTargetPosition, PivotTargetAngle, ExtensionTargetLength;

    private double CurrentPivotAngleZero = 0, CurrentExtensionLengthZero = 0;
    public DoubleSupplier CurrentPivotAngle = () -> Pivot.getCurrentPosition() / 5281.1 * 360 - CurrentPivotAngleZero;
    public DoubleSupplier CurrentExtensionLength = () -> ((ExtensionF.getCurrentPosition() / 384.5) * 360 + CurrentPivotAngle.getAsDouble()) / 2088 * 696 - CurrentExtensionLengthZero;

    private boolean backPedalExtension = false; // whether or not to move extension backwards and then re-extend when pivot is moving

    private boolean justPressedPreset = false, resetExtensionZero = false;

    ElapsedTime ArmLoopTimer, CommandFrameTime, PIDButtonPressTime;
    double FrameRate = 1, ArmLoopTime = 0;
    GamepadEx gamepad, driverGamepad;
    Telemetry telemetry;

    // PID tuning stuff
    boolean PIDButtonPressed = false, PIDIncrementButtonPressed = false;
    double PIDVar = 0, PIDChangeIncrement = 0.01;

    boolean telemetryEnabled = false;

    double CurrentPivotAngleInst = 0; // this speeds up the code a lot by only checking sensors one per update
    double CurrentExtensionLengthInst = 0;


    public ArmSystem(HardwareMap map, Telemetry inputTelemetry) { // Pivot, Extension, Claw, and Wrist initialization
        Pivot = map.get(DcMotorEx.class, "Pivot");
        ExtensionF = map.get(DcMotorEx.class, "ExtensionF");
        ExtensionB = map.get(DcMotorEx.class, "ExtensionB");
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


        telemetry = inputTelemetry;

        PivotPID = new PIDController(0.05, 0, 0, 0, Constants.pivotMaxAngle, 0,
                1, Constants.pivotMaxAngle, 2, true, false, // TODO: speed limiting broken
                CurrentPivotAngle);
        ExtensionPID = new PIDController(0.006, 0, 0, 0, Constants.extensionMaxLength, 0,
                1, 0, 5, true, false,
                CurrentExtensionLength);
    }


    public void initializeArmGamepads() {
        gamepad = SubsystemDataTransfer.operator;
        driverGamepad = SubsystemDataTransfer.driver;
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

    }


    public void updateClawArm() { // VERY IMPORTANT that this needs to be looping constantly
        ArmLoopTimer.reset(); // time it takes the arm for 1 update
        double FrameRate = 1 / (CommandFrameTime.time() / 1000.0); // frame rate of the entire code
        CommandFrameTime.reset();
        telemetry.addData("Code FrameRate:", FrameRate);
        telemetry.addData("Arm System Loop Time:", ArmLoopTime);
        telemetry.addData("Drive System Loop Time:", SubsystemDataTransfer.DrivetrainLoopTime);
        telemetry.addData("HuskyLens Loop Time:", SubsystemDataTransfer.HuskyLensLoopTime);
        telemetry.addLine(" ");

        CurrentPivotAngleInst = CurrentPivotAngle.getAsDouble(); // this speeds up the code a lot by only checking sensors once per update
        CurrentExtensionLengthInst = CurrentExtensionLength.getAsDouble();

        // Manual arm control when controllers active
        if (manualArmControlActive()) {
            backPedalExtension = false;
            Vector2d targetClawPos = getTargetClawPoint();
            double ArmThrottle = 0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            if (FrameRate > 2) {
                // moveArmToPoint(new Vector2d(targetClawPos.x + Constants.maxManualClawSpeedHorizontal * gamepad.getLeftY() / FrameRate, targetClawPos.y + Constants.maxManualClawSpeedVertical * -1 * gamepad.getRightY() / FrameRate));
                moveArmDirectly(PivotTargetAngle + Constants.maxManualPivotSpeed * gamepad.getLeftY() * ArmThrottle / FrameRate,
                        ExtensionTargetLength + Constants.maxManualExtensionSpeed * -1 * gamepad.getRightY() * ArmThrottle / FrameRate);
            }
            // justPressedPreset = false; // also reset button presses
        } /* else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && !justPressedPreset) { // Presets
            resetArm();
            justPressedPreset = true;
        } else justPressedPreset = false;
        */

        // Wrist and Claw
        if (gamepad.getButton(GamepadKeys.Button.Y)) {
            WristTargetAngle = 190;
        } else if (gamepad.getButton(GamepadKeys.Button.X)) {
            WristTargetAngle = 90;
        } else if (gamepad.getButton(GamepadKeys.Button.A)) {
            WristTargetAngle = 0;
        }


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

        // slows down pivot based on how far the extension is extended
        // PivotPID.setPercentMaxSpeed(1 - (1 - Constants.minimumPivotSpeedPercent) * (CurrentExtensionLengthInst / Constants.extensionMaxLength));

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

        // WRIST AND CLAW

        if ((CurrentExtensionLengthInst < 40 || CurrentPivotAngleInst > 45) && WristTargetAngle < 80) { // prevents claw servo from hitting slides
            WristTargetAngle = 90;
        } else if (CurrentPivotAngleInst < 45 && WristTargetAngle > 135) { // prevents huskyLens from hitting hook
            WristTargetAngle = 135;
        }
        setWrist(WristTargetAngle - CurrentPivotAngleInst);

        // right trigger slightly loosens the claw's grip
        Claw.setPosition(ClawTargetPosition - (0.05 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        if (!SubsystemDataTransfer.IMUWorking) telemetry.addLine("IMU HAS STOPPED RESPONDING");
        telemetry.addData("Heading:", Math.toDegrees(SubsystemDataTransfer.getCurrentRobotPose().heading.toDouble()));
        telemetry.addLine("Robot Pose (in) X: " +
                functions.round(SubsystemDataTransfer.getCurrentRobotPose().position.x, 2) + " Y: " +
                functions.round(SubsystemDataTransfer.getCurrentRobotPose().position.y, 2));

        if (telemetryEnabled) { // a lot of telemetry slows the code down so I made it toggleable
            telemetry.addLine(" ");
            Pose2d CurrentClawPosition = getCurrentClawPose(); // avoiding calling this method twice
            telemetry.addLine("Claw Pose X: " +
                    functions.round(CurrentClawPosition.position.x, 2) + " Y: " +
                    functions.round(CurrentClawPosition.position.y, 2) + " H: " +
                    functions.round(getCurrentClawHeight(), 2));
            Pose2d TargetClawPosition = getTargetClawPose(); // avoiding calling this method twice
            telemetry.addLine("Target Claw Pose X: " +
                    functions.round(TargetClawPosition.position.x, 2) + " Y: " +
                    functions.round(TargetClawPosition.position.y, 2) + " H: " +
                    functions.round(getTargetClawHeight(), 2));
            telemetry.addData("Extension Target Length:", ExtensionTargetLength);
            telemetry.addData("Extension Current Length:", CurrentExtensionLengthInst);
            telemetry.addData("Pivot Target Angle:", PivotTargetAngle);
            telemetry.addData("Pivot Current Angle:", CurrentPivotAngleInst);
            telemetry.addData("Is backpedaling Arm:", backPedalExtension);
            // telemetry.addData("Wrist Target Angle:", WristTargetAngle);
            // telemetry.addData("Claw Position:", ClawTargetPosition);
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
            telemetry.addData("Extension Motor Current:", ExtensionF.getCurrent(CurrentUnit.AMPS));

            HuskyLens.Block[] VisionResults = SubsystemDataTransfer.Vision;
            telemetry.addData("HuskyLens block count:", VisionResults.length);
            for (HuskyLens.Block value : VisionResults) {
                telemetry.addLine("ID:" + (value.id) + " x:" + (value.x) + " y:" + (value.y) + // Id, center X, center Y
                        " h:" + (value.height) + " w:" + (value.width)); // height, width,  + " ox" + (value.left) + " oy" + (value.top)  origin X, Origin Y
            }
        }

        tunePIDsWithController(driverGamepad);

        ArmLoopTime = ArmLoopTimer.time(); // updates how long the arm loop took to run
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
            else if (PIDVar == 10) SubsystemDataTransfer.HeadingTargetPID.kP = functions.round(SubsystemDataTransfer.HeadingTargetPID.kP + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 11) SubsystemDataTransfer.HeadingTargetPID.kI = functions.round(SubsystemDataTransfer.HeadingTargetPID.kI + PIDChangeIncrement / 10, 5);
            else if (PIDVar == 12) SubsystemDataTransfer.HeadingTargetPID.kD = functions.round(SubsystemDataTransfer.HeadingTargetPID.kD + PIDChangeIncrement / 10, 5);
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
        return new Vector2d((CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.cos(Math.toRadians(CurrentPivotAngleInst)),
                (CurrentExtensionLengthInst + Constants.retractedExtensionLength) * Math.sin(Math.toRadians(CurrentPivotAngleInst)));
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
        Pose2d CurrentPose = SubsystemDataTransfer.getCurrentRobotPose();
        Vector2d DeltaPose = TargetClawPos.minus(CurrentPose.position); // Vector of where the arm needs to go relative to the robot

        // tell the drivetrain to point towards the target point
        SubsystemDataTransfer.OverrideDrivetrainTargetHeading = Math.toDegrees(Math.atan2(DeltaPose.y, DeltaPose.x));
        SubsystemDataTransfer.OverrideDrivetrainRotation = true;

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

    /*
    public void toggleClaw() {
        if (ClawTargetPosition < 70) ClawTargetPosition = 90;
        else ClawTargetPosition = 0;
    }
    */

    public void toggleTelemetry() { telemetryEnabled = !telemetryEnabled; }

}
