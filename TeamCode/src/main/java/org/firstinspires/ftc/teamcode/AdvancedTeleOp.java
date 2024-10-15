package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
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

        GamepadEx driver = new GamepadEx(gamepad1), operator = new GamepadEx(gamepad2);
        DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(hardwareMap, driver);
        ArmSystem armClaw = new ArmSystem(hardwareMap, operator);

        // buttons
        driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(drivetrain::toggleAbsoluteDriving));



        // always running
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));

        schedule(new RunCommand(telemetry::update));
    }
}
