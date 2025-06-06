package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@Photon
@TeleOp(name="TeleOp")
public class AdvancedTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = true;

        SubsystemData.driver = new GamepadEx(gamepad1);
        SubsystemData.operator = new GamepadEx(gamepad2);

        Pose2d startPose = SubsystemData.CurrentRobotPose;

        Follower follower = new Follower(hardwareMap, startPose, telemetry);
        follower.startTeleopDrive();

        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);


        // BUTTONS
        // Driver controls
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::toggleTelemetry));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(follower::resetIMU));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(follower.getRRDrive()::toggleAbsoluteDriving));

        // Auto Driving Path Selection
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(DriveCommand::setAutoPathToSubmersible));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(DriveCommand::setAutoPathToBasket));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(DriveCommand::setAutoPathToHumanPlayer));
        SubsystemData.driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(DriveCommand::setAutoPathToChamber));

        // Operator controls
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(armSystem::dropSamplePickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(armSystem::toggleBetweenStraightAndFloor));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(armSystem::setWristToBack));

        // Presets
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(armSystem::resetArm));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(armSystem::moveClawToTopRung));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(armSystem::moveClawToHumanPickup));
        SubsystemData.operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(armSystem::moveClawToTopBasket));


        // always running
        follower.setDefaultCommand(new DriveCommand(follower, telemetry, true));
        armSystem.setDefaultCommand(new ArmClawCommand(armSystem));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        waitForStart();
        if (isStopRequested()) return; // prevents crashing if the opmode is stopped in between init and start

        armSystem.resetAndPrepareArm();
    }
}
