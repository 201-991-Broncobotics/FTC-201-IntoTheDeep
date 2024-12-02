package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp(name="TeleOp")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = true;

        SubsystemData.driver = new GamepadEx(gamepad1);
        SubsystemData.operator = new GamepadEx(gamepad2);

        // Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        Pose2d startPose = SubsystemData.LastAutonPose;

        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);


        // BUTTONS
        // Driver controls
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::toggleTelemetry));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(drive::realignHeading));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(drive::toggleAbsoluteDriving));

        // Operator controls
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new InstantCommand(armSystem::pointClaw)).whenReleased(new InstantCommand(armSystem::toggleClaw));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(armSystem::setWristToFloorPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::setWristToStraight));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(armSystem::setWristToBack));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(armSystem::enableLoosenClaw)).whenReleased(new InstantCommand(armSystem::disableLoosenClaw));

        // Presets
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(armSystem::resetArm));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(armSystem::moveClawToTopRung));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(armSystem::moveClawToHumanPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(armSystem::moveClawToTopBasket));


        // always running
        drive.setDefaultCommand(new DriveCommand(drive, telemetry, true));
        armSystem.setDefaultCommand(new ArmClawCommand(armSystem));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        waitForStart();
        armSystem.resetAndPrepareArm();
    }
}
