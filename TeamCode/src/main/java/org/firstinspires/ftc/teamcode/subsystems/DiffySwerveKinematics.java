package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PersistentDataStorage;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;

public class DiffySwerveKinematics extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    private static PoseVelocity2dDual<Time> driveCommand; // for roadrunner
    private static PoseVelocity2d VelocityPose;
    private static double TrackWidth, Voltage; // voltage allows auton to drive at the same power even at low battery
    private static MotorFeedforward FeedForward;

    private static Telemetry telemetry;

    private static ElapsedTime brakeTimer;
    private static double brakeStartTime = 0;

    private static boolean goingToTheRight = true;

    public static boolean DiffyEmergencyRealigning = false;

    private static final ArrayList<PoseVelocity2dDual<Time>> LastCommands = new ArrayList<PoseVelocity2dDual<Time>>();


    public DiffySwerveKinematics(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop, double maxPowerLimit, Telemetry newTelemetry) {
        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down
        telemetry = newTelemetry;

        rightModule = new SwerveModule(newRightTop, newRightBottom, 0); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom, 0);

        telemetry.addData("Current right Diffy Angle:", rightModule.getCurrentAngle());
        telemetry.addData("Current left Diffy Angle:", leftModule.getCurrentAngle());
        telemetry.update();

        brakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        driveCommand = new PoseVelocity2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 1}),
                        new DualNum<>(new double[] {0, 1})),
                new DualNum<>(new double[] {0, 1}));
        VelocityPose = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }


    // only use one of these diffy serve methods at one time as some of the values are shared
    public void driveDifferentialSwerve(double forward, double strafe, double turn, double brake) { // brake will keep the wheels 90 degrees from the direction you are trying to drive or will allow the diffy to "waddle" when the value is between 0 and 1
        double A = -forward - turn; // diffy swerve drive math
        double B = -forward + turn;
        double RightPower = Math.hypot(strafe, A);
        double LeftPower = Math.hypot(strafe, B);

        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = RightPower / max_power; // target motor speeds
        LeftPower = LeftPower / max_power;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));


        // Braking / brake waddling
        if (brake > SubsystemData.lowerTriggerThreshold) {
            RightPower = 0;
            LeftPower = 0;
            RightAngle = RightAngle + 90;
            LeftAngle = LeftAngle + 90;
        } /* else if (brake > SubsystemData.lowerTriggerThreshold) {
            double brakeAngle = (90 - SubsystemData.maxBrakeWaddleAngle) + SubsystemData.maxBrakeWaddleAngle * ((1 / (SubsystemData.upperTriggerThreshold - SubsystemData.lowerTriggerThreshold)) * (brake - SubsystemData.lowerTriggerThreshold));
            if (goingToTheRight) {
                RightAngle = RightAngle + brakeAngle;
                LeftAngle = LeftAngle + brakeAngle;
            } else {
                RightAngle = RightAngle - brakeAngle;
                LeftAngle = LeftAngle - brakeAngle;
            }
            // if modules aren't aligned and timer hasn't reached timeout
            if (!(rightModule.isCloseEnough() && leftModule.isCloseEnough()) && brakeTimer.time() < SubsystemData.SwitchTimeTimeout) brakeStartTime = brakeTimer.time();
            else if (brakeTimer.time() - brakeStartTime > SubsystemData.SwitchTimeMS) { // switch direction and reset timer
                goingToTheRight = !goingToTheRight;
                brakeStartTime = 0;
                brakeTimer.reset();
            }

        } else brakeTimer.reset();
        */


        // actually tell the pod to go to the angle at the power
        if (Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0) {
            rightModule.setModule(RightAngle, RightPower, maxPower);
            leftModule.setModule(LeftAngle, LeftPower, maxPower);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, RightPower, maxPower);
            leftModule.setModule(lastLeftAngle, LeftPower, maxPower);
        }

    }


    public void setDifferentialSwerve(PoseVelocity2dDual<Time> command, PoseVelocity2d currentVelocityPose, double trackWidth, double voltage, MotorFeedforward feedForward) {
        driveCommand = command;
        TrackWidth = trackWidth; // I don't actually use this but roadrunner used to
        Voltage = voltage;
        FeedForward = feedForward;
        updateKinematicDifferentialSwerve();
    }

    public void updateKinematicDifferentialSwerve() {
        // Create an average of the past couple of commands to smooth out the swerve drive's movement
        /*
        LastCommands.add(driveCommand);
        int LastCommandsSize = LastCommands.size();
        if (LastCommandsSize > SubsystemData.CommandBlendingAmount) LastCommands.remove(0);
        LastCommandsSize = LastCommands.size();

        PoseVelocity2dDual<Time> blendedCommand = LastCommands.get(LastCommandsSize - 1); // puts emphasise on the latest command by adding an additional time (definitely not because I needed an command to add to)
        for (PoseVelocity2dDual<Time> c : LastCommands) {
            blendedCommand.plus(c.value());
        }
        blendedCommand = new PoseVelocity2dDual<Time>(new Vector2dDual<>(
                blendedCommand.linearVel.x.div(LastCommandsSize + 1),
                blendedCommand.linearVel.y.div(LastCommandsSize + 1)),
                blendedCommand.angVel.div(LastCommandsSize + 1));

         */

        // telemetry.addLine("Average command X:" + functions.round(blendedCommand.linearVel.x.value(), 3) + " Y:" + functions.round(blendedCommand.linearVel.y.value(), 3) + " A:" + functions.round(blendedCommand.angVel.value(), 3));


        DualNum<Time> forward = driveCommand.linearVel.y;
        DualNum<Time> strafe = driveCommand.linearVel.x;
        DualNum<Time> turn = driveCommand.angVel;

        // set forward values to be no greater than 1 so that the turn part can be at the correct ratio
        if (Math.abs(forward.value()) > 1) forward.div(Math.abs(forward.value()));
        if (Math.abs(strafe.value()) > 1) strafe.div(Math.abs(strafe.value()));

        DualNum<Time> A = forward.times(-1).minus(turn); // diffy swerve drive math
        DualNum<Time> B = forward.times(-1).plus(turn);
        DualNum<Time> RightPower = ((strafe.times(strafe)).plus((A.times(A)))).sqrt();
        DualNum<Time> LeftPower = ((strafe.times(strafe)).plus((B.times(B)))).sqrt();

        double max_power = Math.max(1, Math.max(RightPower.value(), LeftPower.value())); // keeps all motor powers under 1
        DualNum<Time> RightSpeed = RightPower.div(max_power); // target motor speeds
        DualNum<Time> LeftSpeed = LeftPower.div(max_power);
        double RightAngle = Math.toDegrees(Math.atan2(strafe.get(0), A.get(0))); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe.get(0), B.get(0)));

        // telemetry.addLine("KIN: RA: " + functions.round(RightAngle, 3) + " LA: " + functions.round(LeftAngle, 3) + " RP: " + functions.round(RightSpeed.value(), 3) + " LP: " + functions.round(LeftSpeed.value(), 3));

        // tell the pod to go to the angle at the power
        if (Math.abs(strafe.value()) > 0 || Math.abs(forward.value()) > 0 || Math.abs(turn.value()) > 0) {
            //rightModule.setModuleDual(RightAngle, RightSpeed, maxPower, FeedForward, Voltage);
            //leftModule.setModuleDual(LeftAngle, LeftSpeed, maxPower, FeedForward, Voltage);
            rightModule.setModule(RightAngle, RightSpeed.value(), maxPower);
            leftModule.setModule(LeftAngle, LeftSpeed.value(), maxPower);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, 0, maxPower);
            leftModule.setModule(lastLeftAngle, 0, maxPower);
        }
    }


    public void driveTankDiffySwerve(double left, double right) { // because the dumb auton keeps not working

        telemetry.addLine("KIN: RA: " + functions.round(rightModule.getCurrentAngle(), 2) + " LA: " + functions.round(rightModule.getCurrentAngle(), 2) + " RP: " + functions.round(right, 3) + " LP: " + functions.round(left, 3));

        leftModule.setModule(180, left, maxPower);
        rightModule.setModule(180, right, maxPower);
        lastRightAngle = 180;
        lastLeftAngle = 180;
    }


    public void driveTankDiffyTowardsPoint(Pose2d currentPose, Pose2dDual<Arclength> targetPose, MotorFeedforward feedForward, double voltage) {
        Voltage = voltage;
        FeedForward = feedForward;

        double txHeadingDouble = Math.atan2(targetPose.heading.imag.value(), targetPose.heading.real.value());

        double TargetAngleChange = functions.angleDifference(currentPose.heading.toDouble(), txHeadingDouble, 360);
        double turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(txHeadingDouble + 90, 180);

        Pose2d PositionError = new Pose2d(new Vector2d(targetPose.position.x.value() - currentPose.position.x, targetPose.position.y.value() - currentPose.position.y), txHeadingDouble - currentPose.heading.toDouble());

        double distanceFromTargetPose = Math.hypot(PositionError.position.x, PositionError.position.y);

        if (distanceFromTargetPose < SubsystemData.stopHeadingChangeDistanceTolerance) {
            turn = 0; // stop turning
        }
        SubsystemData.TankLandingPID.setTarget(distanceFromTargetPose);
        double forward = -1 * SubsystemData.TankLandingPID.getPower();
        if (SubsystemData.TankDriveAngleSharpness < 1) SubsystemData.TankDriveAngleSharpness = 1;

        telemetry.addData("Forward before sharpness:", forward);
        telemetry.addData("Difference of Angles test:", TargetAngleChange - functions.angleDifference(currentPose.heading.toDouble(), txHeadingDouble, 180));

        // Prevents driving when not pointing the correct direction and reverses forward when facing backward
        double DriveSharpnessCurve = Math.sin(((Math.abs(TargetAngleChange) / 90) - 1) * Math.PI / 2);
        forward = forward * (Math.signum(DriveSharpnessCurve) * Math.abs(Math.pow(DriveSharpnessCurve, SubsystemData.TankDriveAngleSharpness)));

        telemetry.addData("Distance From Target:", distanceFromTargetPose);
        telemetry.addData("Target Angle Change:", TargetAngleChange);
        telemetry.addData("forward:", forward);
        telemetry.addData("turn:", turn);


        // Set the power for each side of the robot
        double RightPower = forward + SubsystemData.turnPercentage * turn;
        double LeftPower = forward - SubsystemData.turnPercentage * turn;
        double driveDivider = Math.max(1, Math.max(RightPower, LeftPower));
        // driveTankDiffySwerve(FeedForward.compute(new DualNum<Time>(new double[] {LeftPower / driveDivider, 3})) / Voltage, FeedForward.compute(new DualNum<Time>(new double[] {RightPower / driveDivider, 3})) / Voltage);
        driveTankDiffySwerve(LeftPower / driveDivider, RightPower / driveDivider);

    }


    public void pointTankDiffyAtAngle(Pose2d currentPose, double angle, MotorFeedforward feedForward, double voltage) {
        Voltage = voltage;
        FeedForward = feedForward;

        // double txHeadingDouble = Math.atan2(targetPose.heading.imag.value(), targetPose.heading.real.value());

        double TargetAngleChange = functions.angleDifference(currentPose.heading.toDouble(), angle, 360);
        double turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(angle + 90, 180);


        telemetry.addData("Target Angle Change:", TargetAngleChange);
        telemetry.addData("forward:", 0);
        telemetry.addData("turn:", turn);


        // Set the power for each side of the robot
        double RightPower = SubsystemData.turnPercentage * turn;
        double LeftPower = -SubsystemData.turnPercentage * turn;
        double driveDivider = Math.max(1, Math.max(RightPower, LeftPower));
        //driveTankDiffySwerve(FeedForward.compute(new DualNum<Time>(new double[] {LeftPower / driveDivider, 3})) / Voltage, FeedForward.compute(new DualNum<Time>(new double[] {RightPower / driveDivider, 3})) / Voltage);
        driveTankDiffySwerve(LeftPower / driveDivider, RightPower / driveDivider);
    }


    public void stopDifferentialSwerve() {
        driveCommand = new PoseVelocity2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 1}),
                        new DualNum<>(new double[] {0, 1})),
                new DualNum<>(new double[] {0, 1}));
        VelocityPose = new PoseVelocity2d(new Vector2d(0, 0), 0);
        SubsystemData.HeadingTargetPID.stopUntilNextUse();
        SubsystemData.AxialPID.stopUntilNextUse();
        SubsystemData.LateralPID.stopUntilNextUse();
        rightModule.fullStopModule();
        leftModule.fullStopModule();
    }


    /*

    double testingAngle = 0;
    double testingTime = 5000; // milliseconds per module of testing
    double testingPower = 0.25;

    boolean testing2ndSide = false;

    double newRightAngle = 0, newLeftAngle = 0;

    double fastestAngularVelocity = 0;

    Pose2d TestingStartPose;

    ElapsedTime TestingTimer;

    ElapsedTime TestingFrameRateTimer;

    public void emergencyAlignDiffySwerve() {
        stopDifferentialSwerve();
        TestingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TestingFrameRateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        fastestAngularVelocity = 0;
        TestingStartPose = SubsystemData.CurrentRobotPose;
        DiffyEmergencyRealigning = true;
    }

    public void updateEmergencyAlignDiffySwerve() {
        double UpdateSpeed = TestingFrameRateTimer.time(); // how many milliseconds between each update
        TestingFrameRateTimer.reset();

        if (testingAngle > 180 && !testing2ndSide) { // switch to other side
            testingAngle = 0;
            fastestAngularVelocity = 0;
            testing2ndSide = true;
        } else if (testingAngle > 180) { // disable emergency aligning and save new values
            testing2ndSide = false;
            testingAngle = 0;
            fastestAngularVelocity = 0;
            DiffyEmergencyRealigning = false;
        }


        if (testingAngle < 180) {
            if (testing2ndSide) { // test left side if on 2nd side
                leftModule.setModule(testingAngle, testingPower, maxPower);
                rightModule.fullStopModule();

                newLeftAngle = testingAngle;
            } else {
                rightModule.setModule(testingAngle, testingPower, maxPower);
                leftModule.fullStopModule();



                newRightAngle = testingAngle;


            }

        } else {
            TestingTimer.reset(); // switch to next angle to test after it has been enough time
        }

        testingAngle += (180 / testingTime) * UpdateSpeed;

    }

     */

}
