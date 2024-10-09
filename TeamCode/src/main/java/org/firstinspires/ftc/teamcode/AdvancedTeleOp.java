package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DifferentialSwerveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        // something

        DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);

        //register(drivetrain);

        //drivetrain.setDefaultCommand(new DifferentialSwerveCommand(drivetrain, driver));



    }
}
