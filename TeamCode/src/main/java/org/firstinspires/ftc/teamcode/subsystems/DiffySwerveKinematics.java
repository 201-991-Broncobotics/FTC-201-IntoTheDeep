package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PersistentDataStorage;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;

public class DiffySwerveKinematics extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    private static PoseVelocity2dDual<Time> driveCommand; // for roadrunner
    private static double TrackWidth, Voltage; // voltage allows auton to drive at the same power even at low battery
    private static MotorFeedforward FeedForward;

    private static Telemetry telemetry;

    private static ElapsedTime brakeTimer;
    private static double brakeStartTime = 0;

    private static boolean goingToTheRight = true;


    private static final ArrayList<PoseVelocity2dDual<Time>> LastCommands = new ArrayList<PoseVelocity2dDual<Time>>();


    public DiffySwerveKinematics(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop, double maxPowerLimit, Telemetry newTelemetry) {
        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down
        telemetry = newTelemetry;
        rightModule = new SwerveModule(newRightTop, newRightBottom, PersistentDataStorage.lastRightDiffyAngle); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom, PersistentDataStorage.lastLeftDiffyAngle);

        brakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        driveCommand = new PoseVelocity2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 1}),
                        new DualNum<>(new double[] {0, 1})),
                new DualNum<>(new double[] {0, 1}));
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
        if (brake > SubsystemData.upperTriggerThreshold) {
            RightPower = 0;
            LeftPower = 0;
            RightAngle = RightAngle + 90;
            LeftAngle = LeftAngle + 90;
        } else if (brake > SubsystemData.lowerTriggerThreshold) {
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
        PersistentDataStorage.lastRightDiffyAngle = rightModule.getCurrentAngle();
        PersistentDataStorage.lastLeftDiffyAngle = leftModule.getCurrentAngle();

    }


    public void setDifferentialSwerve(PoseVelocity2dDual<Time> command, double trackWidth, double voltage, MotorFeedforward feedForward) {
        driveCommand = command;
        TrackWidth = trackWidth; // I don't actually use this but roadrunner used to
        Voltage = voltage;
        FeedForward = feedForward;
        // updateKinematicDifferentialSwerve();
    }

    public void updateKinematicDifferentialSwerve() {
        // Create an average of the past couple of commands to smooth out the swerve drive's movement
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

        telemetry.addLine("Average command X:" + functions.round(blendedCommand.linearVel.x.value(), 3) + " Y:" + functions.round(blendedCommand.linearVel.y.value(), 3) + " A:" + functions.round(blendedCommand.angVel.value(), 3));


        DualNum<Time> forward = blendedCommand.linearVel.y;
        DualNum<Time> strafe = blendedCommand.linearVel.x;
        DualNum<Time> turn = blendedCommand.angVel;

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

        telemetry.addLine("KIN: RA: " + functions.round(RightAngle, 3) + " LA: " + functions.round(LeftAngle, 3) + " RP: " + functions.round(RightSpeed.value(), 3) + " LP: " + functions.round(LeftSpeed.value(), 3));

        // tell the pod to go to the angle at the power
        if (Math.abs(strafe.value()) > 0 || Math.abs(forward.value()) > 0 || Math.abs(turn.value()) > 0) {
            rightModule.setModuleDual(RightAngle, RightSpeed, maxPower, FeedForward, Voltage);
            leftModule.setModuleDual(LeftAngle, LeftSpeed, maxPower, FeedForward, Voltage);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, 0, maxPower);
            leftModule.setModule(lastLeftAngle, 0, maxPower);
        }
        PersistentDataStorage.lastRightDiffyAngle = rightModule.getCurrentAngle();
        PersistentDataStorage.lastLeftDiffyAngle = leftModule.getCurrentAngle();
    }


    public void driveTankDiffySwerve(double left, double right) { // because the dumb auton keeps not working

        telemetry.addLine("KIN: RA: 0 LA: 0 RP: " + functions.round(right, 3) + " LP: " + functions.round(left, 3));

        leftModule.setModule(0, left, maxPower);
        rightModule.setModule(0, right, maxPower);
        lastRightAngle = 0;
        lastLeftAngle = 0;
        PersistentDataStorage.lastRightDiffyAngle = rightModule.getCurrentAngle();
        PersistentDataStorage.lastLeftDiffyAngle = leftModule.getCurrentAngle();
    }


    public void stopDifferentialSwerve() {
        driveCommand = new PoseVelocity2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 1}),
                        new DualNum<>(new double[] {0, 1})),
                new DualNum<>(new double[] {0, 1}));
        SubsystemData.HeadingTargetPID.stopUntilNextUse();
        SubsystemData.AxialPID.stopUntilNextUse();
        SubsystemData.LateralPID.stopUntilNextUse();
        rightModule.fullStopModule();
        leftModule.fullStopModule();
        PersistentDataStorage.lastRightDiffyAngle = rightModule.getCurrentAngle();
        PersistentDataStorage.lastLeftDiffyAngle = leftModule.getCurrentAngle();
    }

}
