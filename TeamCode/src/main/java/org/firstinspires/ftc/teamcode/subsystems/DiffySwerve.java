package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.function.DoubleSupplier;

public class DiffySwerve extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private final DoubleSupplier ForwardSupplier, StrafeSupplier, TurnSupplier, ThrottleSupplier;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    public boolean absoluteDriving;

    private double headingHold;

    MecanumDrive drive;

    ElapsedTime DifferentialSwerveTimer, imuNotWorkingTimer, sinceLastTurnInputTimer;

    Telemetry telemetry;



    public DiffySwerve(MecanumDrive roadrunnerDrive, double maxPowerLimit, Telemetry inputTelemetry, boolean absoluteDrivingEnabled) {
        // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        drive = roadrunnerDrive;
        ForwardSupplier = SubsystemData.driver::getRightY;
        StrafeSupplier = SubsystemData.driver::getRightX;
        TurnSupplier = SubsystemData.driver::getLeftX;
        ThrottleSupplier = () -> SubsystemData.driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        absoluteDriving = absoluteDrivingEnabled;
        // telemetry = telemetryInput;
        drive.updatePoseEstimate(); // update localization
        headingHold = Math.toDegrees(drive.pose.heading.toDouble());
        SubsystemData.HeadingTargetPID = new PIDController(0.012, 0.004, 0.0014, () -> Math.toDegrees(drive.pose.heading.toDouble()));

        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down

        DifferentialSwerveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuNotWorkingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sinceLastTurnInputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry = inputTelemetry;
    }

    public static void setupDiffy(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop) {
        rightModule = new SwerveModule(newRightTop, newRightBottom); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom);
    }

    public void controlDifferentialSwerve() {
        DifferentialSwerveTimer.reset();
        double lastDiffySwerveTime = 0;

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

        double throttleControl = 0.5 + 0.5 * functions.deadZone(ThrottleSupplier.getAsDouble());
        double forward = -1 * functions.deadZone(ForwardSupplier.getAsDouble());
        double strafe = functions.deadZone(StrafeSupplier.getAsDouble());
        double turn = -0.6 * functions.deadZone(TurnSupplier.getAsDouble());
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



    // only use one of these diffy serve methods at one time as some of the values are shared
    public static void driveDifferentialSwerve(double forward, double strafe, double turn) {
        double A = -forward - turn; // diffy swerve drive math
        double B = -forward + turn;
        double RightPower = Math.hypot(strafe, A); // who knows if this will work
        double LeftPower = Math.hypot(strafe, B);

        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = RightPower / max_power; // target motor speeds
        LeftPower = LeftPower / max_power;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));


        // actually tell the pod to go to the angle at the power
        if (Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0) {
            rightModule.setModule(RightAngle, RightPower, maxPower, 1);
            leftModule.setModule(LeftAngle, LeftPower, maxPower, 1);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, RightPower, maxPower, 1);
            leftModule.setModule(lastLeftAngle, LeftPower, maxPower, 1);
        }
    }


    private static PoseVelocity2dDual<Time> driveCommand;
    private static double TrackWidth, Voltage;
    private static MotorFeedforward FeedForward;


    public static void setDifferentialSwerve(PoseVelocity2dDual<Time> command, double trackWidth, double voltage, MotorFeedforward feedForward) {
        driveCommand = command;
        TrackWidth = trackWidth;
        Voltage = voltage;
        FeedForward = feedForward;
        updateKinematicDifferentialSwerve();
    }

    public static void updateKinematicDifferentialSwerve() {
        DualNum<Time> forward = driveCommand.linearVel.y;
        DualNum<Time> strafe = driveCommand.linearVel.x;
        DualNum<Time> turn = driveCommand.angVel.times(TrackWidth);

        DualNum<Time> A = forward.times(-1).minus(turn); // diffy swerve drive math
        DualNum<Time> B = forward.times(-1).plus(turn);
        DualNum<Time> RightPower = ((strafe.times(strafe)).plus((A.times(A)))).sqrt();
        DualNum<Time> LeftPower = ((strafe.times(strafe)).plus((B.times(B)))).sqrt();

        double max_power = Math.max(1, Math.max(RightPower.value(), LeftPower.value())); // keeps all motor powers under 1
        RightPower = RightPower.div(max_power); // target motor speeds
        LeftPower = LeftPower.div(max_power);
        double RightAngle = Math.toDegrees(Math.atan2(strafe.value(), A.value())); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe.value(), B.value()));

        // tell the pod to go to the angle at the power
        if (Math.abs(strafe.value()) > 0 || Math.abs(forward.value()) > 0 || Math.abs(turn.value()) > 0) {
            rightModule.setModule(RightAngle, FeedForward.compute(RightPower), maxPower, Voltage);
            leftModule.setModule(LeftAngle, FeedForward.compute(LeftPower), maxPower, Voltage);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, FeedForward.compute(RightPower), maxPower, Voltage);
            leftModule.setModule(lastLeftAngle, FeedForward.compute(LeftPower), maxPower, Voltage);
        }
    }



    public void toggleAbsoluteDriving() { absoluteDriving = !absoluteDriving; }
    public void absoluteDrivingOff() { absoluteDriving = false; }
    public void absoluteDrivingOn() { absoluteDriving = true; }

    public void realignHeading() { SubsystemData.imuInstance.resetYaw(); }

}
