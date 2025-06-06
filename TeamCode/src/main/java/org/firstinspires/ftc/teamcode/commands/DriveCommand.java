package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.TelemetryLogger.log;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

public class DriveCommand extends CommandBase {

    public static PIDController HeadingTargetPID; // temporarily here so I can tune the PIDs
    public double headingHold;

    Follower drive; // now the implementation for pedro pathing driving

    ElapsedTime DifferentialSwerveTimer, imuNotWorkingTimer, sinceLastTurnInputTimer, LocalizationCorrectionTimer;

    Telemetry telemetry;

    PoseVelocity2d RobotVelocity;

    private double DpadUpTime = 0, DpadRightTime = 0, DpadDownTime = 0, DpadLeftTime = 0;

    private final PathChain PathToHumanPlayerTopPickup, PathToHumanPlayerBottomPickup, PathToHumanPlayerTopDropOff,
            PathToHumanPlayerBottomDropOff, PathToChamberRight, PathToChamberMiddle, PathToChamberLeft,
            PathToBasketTop, PathToBasketBottom, PathToSubmersibleTop, PathToSubmersibleMiddle, PathToSubmersibleBottom;


    public DriveCommand(Follower pedroPathingDrive, Telemetry inputTelemetry, boolean absoluteDrivingEnabled) {
        addRequirements(pedroPathingDrive);

        drive = pedroPathingDrive;
        drive.startTeleopDrive();
        drive.update();

        SubsystemData.lastXDriveVelocity = 0;
        SubsystemData.lastYDriveVelocity = 0;
        SubsystemData.CurrentForwardAcceleration = 0;

        HeadingTargetPID = new PIDController(Settings.HeadingReference.kP, Settings.HeadingReference.kI, Settings.HeadingReference.kD, () -> Math.toDegrees(SubsystemData.CurrentRobotPose.heading.toDouble()));
        HeadingTargetPID.minDifference = Settings.HeadingReference.minDifference; // this is also actively changed in DriveCommand
        HeadingTargetPID.setSettingsTheSameAs(Settings.HeadingReference);

        RobotVelocity = drive.getRRDrive().updatePoseEstimate(); // update localization
        // SubsystemData.RobotVelocity = RobotVelocity;
        SubsystemData.absoluteDriving = absoluteDrivingEnabled;
        SubsystemData.AutoDriving = false;
        headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());

        DifferentialSwerveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuNotWorkingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sinceLastTurnInputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        LocalizationCorrectionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        SubsystemData.CurrentPedroPose = drive.getPose();


        PathToHumanPlayerTopPickup = drive.actionBuilder(new Pose2d(tileCoords(-1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(0.8, 1.7))
                .splineToConstantHeading(tileCoords(1.7, 0.8), Math.toRadians(270))
                .strafeToConstantHeading(tileCoords(1.7, -1))
                .splineToConstantHeading(tileCoords(Settings.DriverAutoTargetCoords.HumanPickupX, Settings.DriverAutoTargetCoords.HumanPickupY), Math.toRadians(270))
                .getPath();

        PathToHumanPlayerBottomPickup = drive.actionBuilder(new Pose2d(tileCoords(-1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(-1.7, -0.8))
                .splineToConstantHeading(tileCoords(-1, -1.6), Math.toRadians(360 + -11.30993247))
                .strafeToConstantHeading(tileCoords(1, -2))
                .splineToConstantHeading(tileCoords(Settings.DriverAutoTargetCoords.HumanPickupX, Settings.DriverAutoTargetCoords.HumanPickupY), Math.toRadians(270))
                .getPath();

        PathToHumanPlayerTopDropOff = drive.actionBuilder(new Pose2d(tileCoords(-1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(0.8, 1.7))
                .splineToConstantHeading(tileCoords(1.7, 0.8), Math.toRadians(270))
                .strafeToConstantHeading(tileCoords(1.7, -1))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.HumanDropOffX, Settings.DriverAutoTargetCoords.HumanDropOffY), Math.toRadians(Settings.DriverAutoTargetCoords.HumanDropOffHeading)), Math.toRadians(Settings.DriverAutoTargetCoords.HumanDropOffHeading))
                .getPath();

        PathToHumanPlayerBottomDropOff = drive.actionBuilder(new Pose2d(tileCoords(-1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(-1.7, -0.8))
                .splineToConstantHeading(tileCoords(-1, -1.6), Math.toRadians(360 + -11.30993247))
                .strafeToConstantHeading(tileCoords(1, -2))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.HumanDropOffX, Settings.DriverAutoTargetCoords.HumanDropOffY), Math.toRadians(Settings.DriverAutoTargetCoords.HumanDropOffHeading)), Math.toRadians(Settings.DriverAutoTargetCoords.HumanDropOffHeading))
                .getPath();

        PathToChamberRight = drive.actionBuilder(new Pose2d(tileCoords(0, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(0.8, 1.7))
                .splineToConstantHeading(tileCoords(1.7, 0.8), Math.toRadians(270))
                .strafeToConstantHeading(tileCoords(1.7, -1.3))
                .splineToConstantHeading(tileCoords(0, -1.3), Math.toRadians(90))
                .getPath();

        PathToChamberMiddle = drive.actionBuilder(new Pose2d(tileCoords(0, -2.5), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(0, -1.3))
                .getPath();

        PathToChamberLeft = drive.actionBuilder(new Pose2d(tileCoords(0, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(-0.8, 1.7))
                .splineToConstantHeading(tileCoords(-1.7, 0.8), Math.toRadians(270))
                .strafeToConstantHeading(tileCoords(-1.7, -1.3))
                .splineToConstantHeading(tileCoords(0, -1.3), Math.toRadians(90))
                .getPath();

        PathToBasketTop = drive.actionBuilder(new Pose2d(tileCoords(1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(-0.8, 1.7))
                .splineToConstantHeading(tileCoords(-1.7, 0.8), Math.toRadians(270))
                .strafeToConstantHeading(tileCoords(-1.7, -1))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.BasketX, Settings.DriverAutoTargetCoords.BasketY), Math.toRadians(225)), Math.toRadians(225))
                .getPath();

        PathToBasketBottom = drive.actionBuilder(new Pose2d(tileCoords(1.7, 1.7), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(1.7, -0.8))
                .splineToConstantHeading(tileCoords(1, -1.6), Math.atan2(-1.75 - -1.6, -0.5 - 1))
                .strafeToConstantHeading(tileCoords(-0.5, -1.75))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.BasketX, Settings.DriverAutoTargetCoords.BasketY), Math.toRadians(225)), Math.toRadians(225))
                .getPath();

        PathToSubmersibleTop = drive.actionBuilder(new Pose2d(tileCoords(1.7, 0), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(1.7, 0.8))
                .splineToConstantHeading(tileCoords(0.8, 1.7), Math.toRadians(180))
                .strafeToConstantHeading(tileCoords(-1.1, 1.7))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.SubmersibleX, Settings.DriverAutoTargetCoords.SubmersibleY), Math.toRadians(0)), Math.toRadians(0))
                .getPath();

        PathToSubmersibleMiddle = drive.actionBuilder(new Pose2d(tileCoords(-2.5, 0), Math.toRadians(0)))
                .strafeToConstantHeading(tileCoords(Settings.DriverAutoTargetCoords.SubmersibleX, Settings.DriverAutoTargetCoords.SubmersibleY))
                .getPath();

        PathToSubmersibleBottom = drive.actionBuilder(new Pose2d(tileCoords(1.7, 0), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(1.7, -0.8))
                .splineToConstantHeading(tileCoords(0.8, -1.7), Math.toRadians(180))
                .strafeToConstantHeading(tileCoords(-1.1, -1.7))
                .splineToLinearHeading(new Pose2d(tileCoords(Settings.DriverAutoTargetCoords.SubmersibleX, Settings.DriverAutoTargetCoords.SubmersibleY), Math.toRadians(0)), Math.toRadians(0))
                .getPath();


        telemetry = inputTelemetry; // only here if I need it for debugging
    }


    @Override
    public void execute() {
        DifferentialSwerveTimer.reset();
        // double lastDiffySwerveTime = 0;

        if (SubsystemData.NeedToRealignHeadingHold) { // reset heading hold when imu is reset
            headingHold = 90;
            if (!SubsystemData.driver.getButton(GamepadKeys.Button.Y)) {
                SubsystemData.needToResetIMU = true;
                SubsystemData.NeedToRealignHeadingHold = false;
            }

        }

        // Makes sure any changes to pid variables get applied to the actual pids
        HeadingTargetPID.setSettingsTheSameAs(Settings.HeadingReference);

        if (SubsystemData.driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            drive.getRRDrive().alignModulesToZero();
        }

        // This will realign the current pose with one of the walls depending on which dpad is being pressed after being held for more than 1 second
        // The timers are all tracked separately so that you can realign x and y at the same time while in a corner
        if (!functions.inUse(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) {
            double currentTime = LocalizationCorrectionTimer.time();

            if (SubsystemData.driver.getButton(GamepadKeys.Button.DPAD_UP)) {
                if (currentTime - DpadUpTime > Settings.DriverLocalizationCorrectionHoldTime) {
                    drive.setPoseRRY(Settings.LocalizationChamberResetY);
                    SubsystemData.LocalizationCoordsAligned[1] = true;
                }
            } else DpadUpTime = LocalizationCorrectionTimer.time();

            if (SubsystemData.driver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                if (currentTime - DpadRightTime > Settings.DriverLocalizationCorrectionHoldTime) {
                    drive.setPoseRRX(Settings.LocalizationRightResetX);
                    SubsystemData.LocalizationCoordsAligned[0] = true;
                }
            } else DpadRightTime = LocalizationCorrectionTimer.time();

            if (SubsystemData.driver.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                if (currentTime - DpadDownTime > Settings.DriverLocalizationCorrectionHoldTime) {
                    drive.setPoseRRY(Settings.LocalizationBottomResetY);
                    SubsystemData.LocalizationCoordsAligned[1] = true;
                }
                //Settings.ArmSystemSettings.startHanging = false;
            } else DpadDownTime = LocalizationCorrectionTimer.time();

            /*
            if (SubsystemData.driver.getButton(GamepadKeys.Button.DPAD_LEFT)) {

                if (currentTime - DpadLeftTime > Settings.DriverLocalizationCorrectionHoldTime) {
                    drive.setPoseRRX(Settings.LocalizationLeftResetX);
                    SubsystemData.LocalizationCoordsAligned[0] = true;
                }



                // Settings.ArmSystemSettings.startHanging = true;
            } else {
                DpadLeftTime = LocalizationCorrectionTimer.time();
            }
         */
        }


        // drive.update(); // update localization


        double DriveXAcceleration = (SubsystemData.CorrectedRobotVelocity.linearVel.x - SubsystemData.lastXDriveVelocity) * SubsystemData.FrameRate;
        double DriveYAcceleration = (SubsystemData.CorrectedRobotVelocity.linearVel.y - SubsystemData.lastYDriveVelocity) * SubsystemData.FrameRate;
        SubsystemData.lastXDriveVelocity = SubsystemData.CorrectedRobotVelocity.linearVel.x;
        SubsystemData.lastYDriveVelocity = SubsystemData.CorrectedRobotVelocity.linearVel.y;
        SubsystemData.CurrentForwardAcceleration = DriveYAcceleration * Math.sin(SubsystemData.CurrentRobotPose.heading.toDouble()) + DriveXAcceleration * Math.cos(SubsystemData.CurrentRobotPose.heading.toDouble());


        YawPitchRollAngles robotOrientation = SubsystemData.IMUAngles;

        // Check if Imu had an ESD event and murdered itself
        if (robotOrientation.getYaw(AngleUnit.DEGREES) == 0 && robotOrientation.getPitch(AngleUnit.DEGREES) == 0 && robotOrientation.getRoll(AngleUnit.DEGREES) == 0) {
            if (imuNotWorkingTimer.time() > 750) {
                SubsystemData.IMUWorking = false;
                SubsystemData.LocalizationCoordsAligned = new boolean[]{false, false};
            }
        } else {
            SubsystemData.IMUWorking = true;
            imuNotWorkingTimer.reset();
        }

        double throttleMagnitude = functions.deadZone(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        double throttleControl = functions.minMaxValue(0, 1, 0.4 + 0.6 * throttleMagnitude - Settings.DriveExtensionDriveReduction * SubsystemData.DriveCurrentExtensionLengthPercent);
        double turnThrottleControl = functions.minMaxValue(0, 1, 0.6 + 0.4 * throttleMagnitude + Settings.DriveExtensionTurnBoost * SubsystemData.DriveCurrentExtensionLengthPercent);
        double forward = -1 * functions.deadZone(SubsystemData.driver.getRightY()); // normalized later using magnitude
        double strafe = functions.deadZone(SubsystemData.driver.getRightX());
        double turn = -turnThrottleControl * Math.pow(SubsystemData.driver.getLeftX(), 3); // normalized for easier driving
        double heading = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
        if (!SubsystemData.absoluteDriving || !SubsystemData.IMUWorking) heading = 90;

        if (!SubsystemData.absoluteDriving) telemetry.addLine("Absolute Driving is off");



        // Auto Driving
        if (SubsystemData.LocalizationCoordsAligned[0] && SubsystemData.LocalizationCoordsAligned[1] && functions.inUse(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) && !functions.inUse(forward) && !functions.inUse(strafe) && !functions.inUse(turn)) {

            if (!SubsystemData.AutoDriving) {
                switch (SubsystemData.CurrentPathSetting) {
                    case 0: // Submersible
                        if (SubsystemData.CurrentRobotPose.position.x < tiles(-0.75) && SubsystemData.CurrentRobotPose.position.y > tiles(-1) && SubsystemData.CurrentRobotPose.position.y < tiles(1)) {
                            drive.followPath(PathToSubmersibleMiddle, true);
                        } else if (SubsystemData.CurrentRobotPose.position.y > tiles(1)) {
                            drive.followPath(PathToSubmersibleTop, true);
                        } else {
                            drive.followPath(PathToSubmersibleBottom, true);
                        }
                        break;
                    case 1: // Human Player

                        if (functions.isPointAboveLine(SubsystemData.CurrentRobotPose.position, new Vector2d(1, -1), new Vector2d(-1, 1)) && SubsystemData.CurrentRobotPose.position.y > tiles(-1)) {
                            drive.followPath(PathToHumanPlayerTopPickup, true);
                        } else {
                            drive.followPath(PathToHumanPlayerBottomPickup, true);
                        }
                        break;
                    case 2: // Chamber
                        if (SubsystemData.CurrentRobotPose.position.y < tiles(-1) && SubsystemData.CurrentRobotPose.position.x > tiles(-0.6) && SubsystemData.CurrentRobotPose.position.x < tiles(0.6)) {
                            drive.followPath(PathToChamberMiddle, true);
                        } else if (SubsystemData.CurrentRobotPose.position.x < tiles(-0.6)) {
                            drive.followPath(PathToChamberLeft, true);
                        } else {
                            drive.followPath(PathToChamberRight, true);
                        }
                        break;
                    case 3: // Basket
                        if (functions.isPointAboveLine(SubsystemData.CurrentRobotPose.position, new Vector2d(-1, -1), new Vector2d(1, 1)) && SubsystemData.CurrentRobotPose.position.y > tiles(-1)) {
                            drive.followPath(PathToBasketTop, true);
                        } else {
                            drive.followPath(PathToBasketBottom, true);
                        }
                        break;
                }
            }

            SubsystemData.AutoDriving = true;
            headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
            SubsystemData.OverrideDrivetrainRotation = false;
            SubsystemData.HoldClawFieldPos = false;

            SubsystemData.AutoDrivingPower = functions.deadZoneNormalized(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), 0.1);

        } else { // Not Auto Driving
            if (SubsystemData.AutoDriving) drive.startTeleopDrive();
            SubsystemData.AutoDriving = false;


            // convert to vector and normalize values to make it easier for the driver to control
            double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
            double joystickMagnitude = Math.hypot(strafe, forward);
            double drivePower = Math.pow(joystickMagnitude, 3); //   // Math.abs(joystickMagnitude) * joystickMagnitude


            if (SubsystemData.driver.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) headingHold = 90;


            if (!functions.inUse(turn) && !SubsystemData.NeedToRealignHeadingHold) { // hold robot orientation or point at claw target when driver isn't driving
                if (functions.inUse(SubsystemData.OperatorTurningPower) && !functions.inUse(forward) && !functions.inUse(strafe)) {
                    sinceLastTurnInputTimer.reset();
                    turn = SubsystemData.OperatorTurningPower; // operator can turn robot if driver isn't currently
                    headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
                    SubsystemData.HoldClawFieldPos = false;

                } else if (SubsystemData.IMUWorking && sinceLastTurnInputTimer.time() > 500 && SubsystemData.absoluteDriving) { // also disables heading correction with absolute driving
                    // otherwise hold current heading if no driver input for some time and imu is working
                    // auto aim
                    //if (SubsystemData.OverrideDrivetrainRotation) headingHold = headingHold - SubsystemData.AutoAimHeading;
                    if (SubsystemData.OverrideDrivetrainRotation) headingHold = SubsystemData.OverrideDrivetrainTargetHeading;

                    // This stops the headingPID from turning while at low drive powers because otherwise it causes the swerve modules to go crazy
                    if (Math.abs(drivePower) < 0.1 && !(drivePower == 0)) HeadingTargetPID.minDifference = Settings.slowMovingHeadingPIDMinDifference;

                    turn = -1 * HeadingTargetPID.getPowerWrapped(headingHold, 360);

                } else headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble()); // keep heading hold updating while robot finishes rotating from manual control

            } else {
                sinceLastTurnInputTimer.reset();
                SubsystemData.OverrideDrivetrainRotation = false;
                turn = turn * throttleControl;
                headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
                SubsystemData.HoldClawFieldPos = false;
            }

            SubsystemData.HeadHoldTarget = headingHold;

            driveDirection = functions.angleDifference(0, driveDirection - heading + 90, 360);

            double strafeControl = -1 * Math.cos(Math.toRadians(driveDirection)) * drivePower * throttleControl;

            drive.setTeleOpMovementVectors(
                    Math.sin(Math.toRadians(driveDirection)) * drivePower * throttleControl,
                    strafeControl,
                    turn /* + strafeControl * Constants.strafeRotationGain */);

        }

        drive.update();

        SubsystemData.DrivetrainLoopTime = DifferentialSwerveTimer.time(); // logs time it took to run from top to bottom
    }


    public static void setAutoPathToSubmersible() {
        SubsystemData.CurrentPathSetting = 0;
        SubsystemData.AutoDriving = false;
    }
    public static void setAutoPathToHumanPlayer() {
        SubsystemData.CurrentPathSetting = 1;
        SubsystemData.AutoDriving = false;
    }
    public static void setAutoPathToChamber() {
        SubsystemData.CurrentPathSetting = 2;
        SubsystemData.AutoDriving = false;
    }
    public static void setAutoPathToBasket() {
        SubsystemData.CurrentPathSetting = 3;
        SubsystemData.AutoDriving = false;
    }

}