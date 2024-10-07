package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DifferentialSwerveCommand extends CommandBase {




    private final DoubleSupplier right_joystick_x, right_joystick_y, left_joystick_x, throttle;

    private double driveDirection, joystickMagnitude, drivePower, throttleControl;


    private final boolean absoluteDriving = false;

    public DifferentialSwerveCommand(DifferentialSwerveDrivetrain drivetrain, GamepadEx driver) {
        addRequirements(drivetrain);
        this.right_joystick_x = () -> 1 * driver.getRightX();
        this.right_joystick_y = () -> 1 * driver.getRightY();
        this.left_joystick_x = () -> 1 * driver.getLeftX();

        this.throttle = () -> 1 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void initialize() { } // when command first starts

    @Override
    public void execute() { // what is run constantly
        double right_joystick_x_value = right_joystick_x.getAsDouble();
        double right_joystick_y_value = right_joystick_y.getAsDouble();
        double left_joystick_x_value = left_joystick_x.getAsDouble();
        double right_trigger_value = throttle.getAsDouble();
        driveDirection = Math.toDegrees(Math.atan2(right_joystick_y_value, right_joystick_x_value));
        joystickMagnitude = Math.hypot(right_joystick_x_value, right_joystick_y_value);
        drivePower = Math.abs(joystickMagnitude) * joystickMagnitude; // Makes it easier to control the robot
        if (right_trigger_value > 0) {
            throttleControl = 0.5 + right_trigger_value / 2;
        } else throttleControl = 0.5;


        //if (absoluteDriving) {
        //    driveDirection = functions.angleDifference(driveDirection + heading, 0, 360);
        //}

        drivetrain.driveDifferentialSwerve(driveDirection, drivePower, -0.6 * Math.abs(left_joystick_x_value) * left_joystick_x_value, throttleControl);
    }

    @Override
    public void end(boolean interrupted) { } // what happens at the end

    @Override
    public boolean isFinished() {
        return false;
    }
}
