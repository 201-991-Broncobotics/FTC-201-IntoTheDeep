package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;


@TeleOp(name="CommandManualFeedbackTuner")
public class CommandManualFeedbackTuner extends CommandOpMode {

    public static double DISTANCE = 20;

    @Override
    public void initialize() {

        // assume a position of 0, 0, 0 is in the exact center of the field pointing away from audience

        SubsystemData.driver = new GamepadEx(gamepad1);
        SubsystemData.operator = new GamepadEx(gamepad2);

        Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);

        // This keeps the stuff updating in the background of auton instead of just when it needs to be used
        drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        armSystem.setDefaultCommand(new ArmClawCommand(armSystem));

        // schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        // setup roadrunner trajectories

        drive.updatePoseEstimate();

        waitForStart();

        // schedule auton commands
        DriveAutonCommand.runningActions.add(new SequentialAction(
                drive.actionBuilder(startPose)
                        .strafeToConstantHeading(new Vector2d(0, 10))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(new Vector2d(0, 0))
                        .waitSeconds(0.5)
                        //.turnTo(Math.toRadians(270))
                        //.waitSeconds(1)
                        .build()
        ));

    }

}
