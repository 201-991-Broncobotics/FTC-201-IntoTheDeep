package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;

public class DifferentialSwerveDrivetrain {

    private final SwerveModule rightModule, leftModule;

    private double lastRightAngle = 0.0, lastLeftAngle = 0.0;

    public DifferentialSwerveDrivetrain(HardwareMap map) {
        rightModule = new SwerveModule(map, "R1", "R2"); // rotation encoder assumed to be on the top motors
        leftModule = new SwerveModule(map, "R3", "R4");
    }


    public void driveDifferentialSwerve(double direction, double magnitude, double turn, double throttle) {
        double forward = Math.sin(Math.toRadians(direction)) * magnitude; // convert vector to x and y
        double strafe = Math.cos(Math.toRadians(direction)) * magnitude;

        double A = -forward - turn; // diffy swerve drive math
        double B = -forward + turn;
        double RightPower = Math.sqrt(strafe * strafe + A * A);
        double LeftPower = Math.sqrt(strafe * strafe + B * B);

        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = RightPower / max_power; // target motor speeds
        LeftPower = LeftPower / max_power;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));

        // actually tell the pod to go to the angle at the power
        if (Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0) {
            rightModule.setModule(RightAngle, RightPower, throttle);
            leftModule.setModule(LeftAngle, LeftPower, throttle);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, 0, throttle);
            leftModule.setModule(lastLeftAngle, 0, throttle);
        }
    }


    public void driveDifferentialSwerve(double direction, double magnitude, double turn) { // without throttle variable
        driveDifferentialSwerve(direction, magnitude, turn, 1);
    }

}
