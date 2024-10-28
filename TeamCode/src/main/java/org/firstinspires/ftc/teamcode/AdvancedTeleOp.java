package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));


        SubsystemDataTransfer.driver = new GamepadEx(gamepad1);
        SubsystemDataTransfer.operator = new GamepadEx(gamepad2);

        DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(hardwareMap, currentPose, 0.8);
        ArmSystem armClaw = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // BUTTONS

        SubsystemDataTransfer.driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new InstantCommand(armClaw::toggleTelemetry));
        SubsystemDataTransfer.driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(drivetrain::toggleAbsoluteDriving));

        // Claw
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::closeClaw));
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::openClaw));

        // Presets
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(new InstantCommand(armClaw::resetArm));
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(new InstantCommand(armClaw::moveClawToTopRung));
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(new InstantCommand(armClaw::moveClawToHumanPickup));
        SubsystemDataTransfer.operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).toggleWhenPressed(new InstantCommand(armClaw::moveClawToTopBasket));


        // always running
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled
    }
}
