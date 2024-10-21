package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));

        GamepadEx driver = new GamepadEx(gamepad1), operator = new GamepadEx(gamepad2);
        DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(hardwareMap, currentPose, driver);
        ArmSystem armClaw = new ArmSystem(hardwareMap, operator, telemetry, driver);

        // buttons
        driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(drivetrain::toggleAbsoluteDriving));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::closeClaw));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new InstantCommand(armClaw::openClaw));

        // always running
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));

        schedule(new RunCommand(telemetry::update));
    }
}
