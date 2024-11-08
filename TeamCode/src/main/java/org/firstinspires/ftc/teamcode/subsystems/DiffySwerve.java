package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;

public class DiffySwerve extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    private static PoseVelocity2dDual<Time> driveCommand;
    private static double TrackWidth, Voltage;
    private static MotorFeedforward FeedForward;


    public DiffySwerve(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop, double maxPowerLimit) {
        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down
        rightModule = new SwerveModule(newRightTop, newRightBottom); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom);
    }


    // only use one of these diffy serve methods at one time as some of the values are shared
    public void driveDifferentialSwerve(double forward, double strafe, double turn) {
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


    public void setDifferentialSwerve(PoseVelocity2dDual<Time> command, double trackWidth, double voltage, MotorFeedforward feedForward) {
        driveCommand = command;
        TrackWidth = trackWidth;
        Voltage = voltage;
        FeedForward = feedForward;
        updateKinematicDifferentialSwerve();
    }

    public void updateKinematicDifferentialSwerve() {
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

}
