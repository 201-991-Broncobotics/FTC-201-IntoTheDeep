package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DiffySwerve;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        SubsystemData.driver = new GamepadEx(gamepad1);
        SubsystemData.operator = new GamepadEx(gamepad2);

        // HuskyLensCamera huskyLensSystem = new HuskyLensCamera(hardwareMap);
        DiffySwerve drivetrain = new DiffySwerve(drive, 0.8, telemetry, true);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);


        // BUTTONS

        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::toggleTelemetry));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(drivetrain::realignHeading));
        //SubsystemData.driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(drivetrain::toggleAbsoluteDriving));

        // Claw
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(armSystem::toggleClaw));

        // Presets
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(armSystem::resetArm));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(armSystem::moveClawToTopRung));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(armSystem::moveClawToHumanPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(armSystem::moveClawToTopBasket));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(armSystem::setWristToFloorPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::setWristToCenter));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(armSystem::setWristToBasket));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(armSystem::enableLoosenClaw)).whenReleased(new InstantCommand(armSystem::disableLoosenClaw));


        // always running
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armSystem.setDefaultCommand(new ArmClawCommand(armSystem));
        // huskyLensSystem.setDefaultCommand(new HuskyLensCommand(huskyLensSystem));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled
    }
}
