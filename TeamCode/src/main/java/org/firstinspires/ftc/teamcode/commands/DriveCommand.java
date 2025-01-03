package org.firstinspires.ftc.teamcode.commands;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

public class DriveCommand extends CommandBase {

    public double headingHold;

    Follower drive; // now the implementation for pedro pathing driving

    ElapsedTime DifferentialSwerveTimer, imuNotWorkingTimer, sinceLastTurnInputTimer;

    Telemetry telemetry;

    PoseVelocity2d RobotVelocity;
    double lastDriveVelocity = 0, lastTurnVelocity = 0;

    public DriveCommand(Follower pedroPathingDrive, Telemetry inputTelemetry, boolean absoluteDrivingEnabled) {
        addRequirements(pedroPathingDrive);

        drive = pedroPathingDrive;

        RobotVelocity = drive.getRRDrive().updatePoseEstimate(); // update localization
        SubsystemData.RobotVelocity = RobotVelocity;
        SubsystemData.absoluteDriving = absoluteDrivingEnabled;
        headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());

        DifferentialSwerveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuNotWorkingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sinceLastTurnInputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Pose PedroPose = drive.getPose();
        SubsystemData.CurrentPedroPose = new Pose2d(new Vector2d(PedroPose.getX(), PedroPose.getY()), PedroPose.getHeading());

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

            SubsystemData.HighDriveVel = 0; // also reset the high drive/turn vel/accel counts
            SubsystemData.HighDriveAccel = 0;
            SubsystemData.LowDriveAccel = 0;
            SubsystemData.HighAngVel = 0;
            SubsystemData.HighAngAccel = 0;
        }

        //telemetry.addData("Diffy Point 1:", DifferentialSwerveTimer.time() - lastDiffySwerveTime);
        //lastDiffySwerveTime = DifferentialSwerveTimer.time();

        drive.update(); // update localization

        RobotVelocity = SubsystemData.RobotVelocity;
        SubsystemData.CurrentRobotPose = drive.getRRDrive().pose;
        Pose PedroPose = drive.getPose();
        SubsystemData.CurrentPedroPose = new Pose2d(new Vector2d(PedroPose.getX(), PedroPose.getY()), PedroPose.getHeading());

        double DriveVelocity = Math.hypot(RobotVelocity.linearVel.x, RobotVelocity.linearVel.y);
        if (DriveVelocity > SubsystemData.HighDriveVel) SubsystemData.HighDriveVel = DriveVelocity;
        if (Math.abs(RobotVelocity.angVel) > SubsystemData.HighAngVel) SubsystemData.HighAngVel = Math.abs(RobotVelocity.angVel);
        double DriveAcceleration = (DriveVelocity - lastDriveVelocity) * SubsystemData.FrameRate;
        lastDriveVelocity = DriveVelocity;
        double TurnAcceleration = (RobotVelocity.angVel - lastTurnVelocity) * SubsystemData.FrameRate;
        lastTurnVelocity = RobotVelocity.angVel;
        if (DriveAcceleration > SubsystemData.HighDriveAccel) SubsystemData.HighDriveAccel = DriveAcceleration;
        if (DriveAcceleration < SubsystemData.LowDriveAccel) SubsystemData.LowDriveAccel = DriveAcceleration;
        if (Math.abs(TurnAcceleration) > SubsystemData.HighAngAccel) SubsystemData.HighAngAccel = Math.abs(TurnAcceleration);


        YawPitchRollAngles robotOrientation = SubsystemData.IMUAngles;

        // Check if Imu had an ESD event and murdered itself
        if (robotOrientation.getYaw(AngleUnit.DEGREES) == 0 && robotOrientation.getPitch(AngleUnit.DEGREES) == 0 && robotOrientation.getRoll(AngleUnit.DEGREES) == 0) {
            if (imuNotWorkingTimer.time() > 1200) SubsystemData.IMUWorking = false;
        } else {
            SubsystemData.IMUWorking = true;
            imuNotWorkingTimer.reset();
        }

        double throttleMagnitude = functions.deadZone(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        double throttleControl = 0.4 + 0.6 * throttleMagnitude;
        double turnThrottleControl = 0.6 + 0.4 * throttleMagnitude;
        double forward = -1 * functions.deadZone(SubsystemData.driver.getRightY()); // normalized later using magnitude
        double strafe = functions.deadZone(SubsystemData.driver.getRightX());
        double turn = -turnThrottleControl * Math.pow(functions.deadZone(SubsystemData.driver.getLeftX()), 3); // normalized for easier driving
        double heading = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
        if (!SubsystemData.absoluteDriving || !SubsystemData.IMUWorking) heading = 90;

        if (!SubsystemData.absoluteDriving) telemetry.addLine("Absolute Driving is off");


        /*
        // Auto Aiming
        if (SubsystemData.driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05 && Math.abs(forward) < Constants.controllerDeadZone && Math.abs(strafe) < Constants.controllerDeadZone) {
            if (SubsystemData.CameraTargetPixelsY > 100) { // validates target
                if (SubsystemData.AutoAimingForWall) {
                    forward = -1 * SubsystemData.AutoAimForwardGain * (SubsystemData.CameraTargetsPixelsWidth - ) / 160;
                    strafe = SubsystemData.AutoAimStrafeGain * SubsystemData.CameraTargetPixelsX / 160;
                }
            }
        }
         */

        // convert to vector and normalize values to make it easier for the driver to control
        double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
        double joystickMagnitude = Math.hypot(strafe, forward);
        double drivePower = Math.abs(joystickMagnitude) * joystickMagnitude; //  Math.pow(joystickMagnitude, 3)


        if (SubsystemData.driver.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) headingHold = 90;


        if (!functions.inUse(turn) && !SubsystemData.NeedToRealignHeadingHold) { // hold robot orientation or point at claw target when driver isn't driving
            if (functions.inUse(SubsystemData.OperatorTurningPower) && !functions.inUse(forward) && !functions.inUse(strafe)) {
                sinceLastTurnInputTimer.reset();
                turn = SubsystemData.OperatorTurningPower; // operator can turn robot if driver isn't currently
                headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
                SubsystemData.HoldClawFieldPos = false;

            } else if (SubsystemData.IMUWorking && sinceLastTurnInputTimer.time() > 600 && SubsystemData.absoluteDriving) { // also disables heading correction with absolute driving
                // otherwise hold current heading if no driver input for some time and imu is working
                // auto aim
                //if (SubsystemData.OverrideDrivetrainRotation) headingHold = headingHold - SubsystemData.AutoAimHeading;
                if (SubsystemData.OverrideDrivetrainRotation) headingHold = SubsystemData.OverrideDrivetrainTargetHeading;

                // This stops the headingPID from turning while at low drive powers because otherwise it causes the swerve modules to go crazy
                if (Math.abs(drivePower) < 0.1 && !(drivePower == 0)) SubsystemData.HeadingTargetPID.minDifference = 3;
                else SubsystemData.HeadingTargetPID.minDifference = 0.5;

                turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(headingHold, 360);
                telemetry.addLine("Heading Correction is Active");

            } else headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble()); // keep heading hold updating while robot finishes rotating from manual control

        } else {
            sinceLastTurnInputTimer.reset();
            SubsystemData.OverrideDrivetrainRotation = false;
            turn = turn * throttleControl;
            headingHold = Math.toDegrees(drive.getRRDrive().pose.heading.toDouble());
            SubsystemData.HoldClawFieldPos = false;
        }

        SubsystemData.HeadHoldTarget = headingHold;

        driveDirection = functions.angleDifference(driveDirection - heading, 0, 360);

        // convert vector to x and y and multiple by throttle (i think i switched forward and strafe by accident like 20 years ago)
        drive.setTeleOpMovementVectors(
                Math.cos(Math.toRadians(driveDirection)) * drivePower * throttleControl,
                Math.sin(Math.toRadians(driveDirection)) * drivePower * throttleControl,
                turn);
        drive.update();

        SubsystemData.DrivetrainLoopTime = DifferentialSwerveTimer.time(); // logs time it took to run from top to bottom
    }

}