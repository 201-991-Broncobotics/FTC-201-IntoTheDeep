package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@Autonomous(name="RedRightSampleCollection", group="Iterative Opmode")
public class RedRightSampleCollection extends CommandOpMode {

    @Override
    public void initialize() {

        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));

        DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(hardwareMap, currentPose, 0.8);
        ArmSystem armClaw = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);


        // always running
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled
    }
}
