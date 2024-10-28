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

        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);
        DiffySwerve drivetrain = new DiffySwerve(drive, 0.8);
        ArmSystem armClaw = new ArmSystem(hardwareMap, telemetry);


        // BUTTONS

        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new InstantCommand(armClaw::toggleTelemetry));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(drivetrain::toggleAbsoluteDriving));

        // Claw
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::closeClaw));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::openClaw));

        // Presets
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(new InstantCommand(armClaw::resetArm));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(new InstantCommand(armClaw::moveClawToTopRung));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(new InstantCommand(armClaw::moveClawToHumanPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).toggleWhenPressed(new InstantCommand(armClaw::moveClawToTopBasket));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new InstantCommand(armClaw::setWristToFloorPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(armClaw::setWristToCenter));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(new InstantCommand(armClaw::setWristToBasket));


        // always running
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled
    }
}
