package org.firstinspires.ftc.teamcode.commands;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

public class DriveCommand extends CommandBase {

    public boolean absoluteDriving;

    private double headingHold;

    DifferentialSwerveDrive drive;

    ElapsedTime DifferentialSwerveTimer, imuNotWorkingTimer, sinceLastTurnInputTimer;

    Telemetry telemetry;

    public DriveCommand(DifferentialSwerveDrive roadrunnerDrive, Telemetry inputTelemetry, boolean absoluteDrivingEnabled) {
        addRequirements(roadrunnerDrive);
        // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        drive = roadrunnerDrive;
        drive.updatePoseEstimate(); // update localization
        absoluteDriving = absoluteDrivingEnabled;
        headingHold = Math.toDegrees(drive.pose.heading.toDouble());

        DifferentialSwerveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuNotWorkingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sinceLastTurnInputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry = inputTelemetry; // only here if I need it for debugging
    }


    @Override
    public void execute() {
        DifferentialSwerveTimer.reset();
        // double lastDiffySwerveTime = 0;

        //telemetry.addData("Diffy Point 1:", DifferentialSwerveTimer.time() - lastDiffySwerveTime);
        //lastDiffySwerveTime = DifferentialSwerveTimer.time();

        drive.updatePoseEstimate(); // update localization
        SubsystemData.CurrentRobotPose = drive.pose;

        YawPitchRollAngles robotOrientation = SubsystemData.IMUAngles;

        // Check if Imu had an ESD event and murdered itself
        if (robotOrientation.getYaw(AngleUnit.DEGREES) == 0 && robotOrientation.getPitch(AngleUnit.DEGREES) == 0 && robotOrientation.getRoll(AngleUnit.DEGREES) == 0) {
            if (imuNotWorkingTimer.time() > 2500) {
                SubsystemData.IMUWorking = false;
            }
        } else {
            SubsystemData.IMUWorking = true;
            imuNotWorkingTimer.reset();
        }

        double throttleControl = 0.5 + 0.5 * functions.deadZone(SubsystemData.driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        double forward = -1 * functions.deadZone(SubsystemData.driver.getRightY());
        double strafe = functions.deadZone(SubsystemData.driver.getRightX());
        double turn = -0.6 * functions.deadZone(SubsystemData.driver.getLeftX());
        double heading = Math.toDegrees(drive.pose.heading.toDouble());
        if (!absoluteDriving || !SubsystemData.IMUWorking) heading = 90;

        if (!functions.inUse(turn)) { // hold robot orientation or point at claw target when driver isn't turning
            // if (SubsystemData.OverrideDrivetrainRotation) headingHold = SubsystemData.OverrideDrivetrainTargetHeading;
            if (functions.inUse(SubsystemData.OperatorTurningPower)) {
                sinceLastTurnInputTimer.reset();
                turn = SubsystemData.OperatorTurningPower; // operator can turn robot if driver isn't currently
                headingHold = Math.toDegrees(drive.pose.heading.toDouble());
                SubsystemData.HoldClawFieldPos = false;

            } else if (SubsystemData.IMUWorking && sinceLastTurnInputTimer.time() > 300) {
                // otherwise hold current heading if no driver input for some time and imu is working
                if (SubsystemData.OverrideDrivetrainRotation) { // auto aim
                    headingHold = headingHold - SubsystemData.AutoAimHeading;
                }
                turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(headingHold, 360);
            } else headingHold = Math.toDegrees(drive.pose.heading.toDouble());

        } else {
            sinceLastTurnInputTimer.reset();
            SubsystemData.OverrideDrivetrainRotation = false;
            turn = turn * throttleControl;
            headingHold = Math.toDegrees(drive.pose.heading.toDouble());
            SubsystemData.HoldClawFieldPos = false;
        }

        SubsystemData.HeadingHold = headingHold; // for telemetry

        // convert to vector and normalize values to make it easier for the driver to control
        double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
        double joystickMagnitude = Math.hypot(strafe, forward);
        double drivePower = Math.abs(joystickMagnitude) * joystickMagnitude;

        driveDirection = functions.angleDifference(driveDirection - heading, 0, 360);

        double forwardNorm = Math.sin(Math.toRadians(driveDirection)) * drivePower; // convert vector to x and y
        double strafeNorm = Math.cos(Math.toRadians(driveDirection)) * drivePower;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardNorm * throttleControl,
                strafeNorm * throttleControl), turn));

        SubsystemData.DrivetrainLoopTime = DifferentialSwerveTimer.time(); // logs time it took to run from top to bottom
    }

}