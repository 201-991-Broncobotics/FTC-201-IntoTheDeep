package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DifferentialSwerveDrivetrain extends SubsystemBase {

    private final SwerveModule rightModule, leftModule;

    private final DoubleSupplier ForwardSupplier, StrafeSupplier, TurnSupplier, ThrottleSupplier;

    private double lastRightAngle = 0.0, lastLeftAngle = 0.0;

    public boolean absoluteDriving = true;

    public DifferentialSwerveDrivetrain(HardwareMap map, GamepadEx gamepad) {
        rightModule = new SwerveModule(map, "R2", "R1"); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(map, "R3", "R4");
        ForwardSupplier = gamepad::getLeftY;
        StrafeSupplier = gamepad::getLeftX;
        TurnSupplier = gamepad::getRightX;
        ThrottleSupplier = () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }


    public void controlDifferentialSwerve() {
        double forward = ForwardSupplier.getAsDouble(); // get initial values
        double strafe = StrafeSupplier.getAsDouble();
        double turn = TurnSupplier.getAsDouble();
        double throttle = ThrottleSupplier.getAsDouble();
        double heading = 0; // TODO: needs to be set to the correct twist value from roadrunner

        // normalize values to make it easier for the driver to control
        double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
        double joystickMagnitude = Math.hypot(strafe, forward);
        double drivePower = Math.abs(joystickMagnitude) * joystickMagnitude;
        double throttleControl = 0.5 + throttle / 2;

        if (absoluteDriving) {
            driveDirection = functions.angleDifference(driveDirection + heading, 0, 360);
        }

        driveDifferentialSwerve(driveDirection, drivePower, -0.4 * Math.abs(turn) * turn, throttleControl);
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


    public void toggleAbsoluteDriving() {
        absoluteDriving = !absoluteDriving;
    }


    public void driveDifferentialSwerve(double direction, double magnitude, double turn) { // without throttle variable
        driveDifferentialSwerve(direction, magnitude, turn, 1);
    }


}
