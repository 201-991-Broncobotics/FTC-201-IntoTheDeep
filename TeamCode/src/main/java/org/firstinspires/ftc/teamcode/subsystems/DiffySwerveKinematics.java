package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

public class DiffySwerveKinematics extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    private static PoseVelocity2dDual<Time> driveCommand; // for roadrunner
    private static double TrackWidth, Voltage; // voltage allows auton to drive at the same power even at low battery
    private static MotorFeedforward FeedForward;

    private static Telemetry telemetry;


    public DiffySwerveKinematics(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop, double maxPowerLimit, Telemetry newTelemetry) {
        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down
        telemetry = newTelemetry;
        rightModule = new SwerveModule(newRightTop, newRightBottom); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom);
    }


    // only use one of these diffy serve methods at one time as some of the values are shared
    public void driveDifferentialSwerve(double forward, double strafe, double turn) {
        double A = -forward - turn; // diffy swerve drive math
        double B = -forward + turn;
        double RightPower = Math.hypot(strafe, A);
        double LeftPower = Math.hypot(strafe, B);

        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = RightPower / max_power; // target motor speeds
        LeftPower = LeftPower / max_power;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));

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


    public void setDifferentialSwerve(PoseVelocity2dDual<Time> command, double trackWidth, double voltage, MotorFeedforward feedForward) {
        driveCommand = command;
        TrackWidth = trackWidth; // I don't actually use this but roadrunner used to
        Voltage = voltage;
        FeedForward = feedForward;
        // updateKinematicDifferentialSwerve();
    }

    public void updateKinematicDifferentialSwerve() {
        DualNum<Time> forward = driveCommand.linearVel.y; // roadrunner is stupid and has these flipped
        DualNum<Time> strafe = driveCommand.linearVel.x;
        DualNum<Time> turn = driveCommand.angVel;

        // set forward values to be no greater than 1 so that the turn part can be at the correct ratio
        if (Math.abs(forward.value()) > 1) {
            double ForwardDiv = Math.abs(forward.value());
            forward = new DualNum<Time>(new double[] {forward.get(0) / ForwardDiv, forward.get(1) / ForwardDiv});
        }
        if (Math.abs(strafe.value()) > 1) {
            double StrafeDiv = strafe.value();
            strafe = new DualNum<Time>(new double[] {strafe.get(0) / StrafeDiv, strafe.get(1) / StrafeDiv});
        }

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
    }

    public void stopDifferentialSwerve() {
        driveCommand = new PoseVelocity2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 0}),
                        new DualNum<>(new double[] {0, 0})),
                new DualNum<>(new double[] {0, 0}));
        SubsystemData.HeadingTargetPID.stopUntilNextUse();
        rightModule.fullStopModule();
        leftModule.fullStopModule();
    }

}
