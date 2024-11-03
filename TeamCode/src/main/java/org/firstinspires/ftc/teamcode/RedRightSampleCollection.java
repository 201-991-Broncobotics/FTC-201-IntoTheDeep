package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DiffySwerve;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;


@Autonomous(name="RedRightSampleCollection", group="Red Side")
public class RedRightSampleCollection extends CommandOpMode {

    @Override
    public void initialize() {

        // assume a position of 0, 0, 0 is in the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(functions.tiles(0.5), functions.tiles(-3) + 7.09), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DiffySwerve drivetrain = new DiffySwerve(drive, 0.8, telemetry, false);
        ArmSystem armClaw = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // always running
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
        armClaw.setDefaultCommand(new ArmClawCommand(armClaw));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToRungs = drive.actionBuilder(startPose)
                .strafeToConstantHeading(functions.tileCoords(0.2, -1.6))
                .waitSeconds(2);

        TrajectoryActionBuilder DriveToFirstSample = DriveToRungs.fresh()
                .setTangent(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(functions.tileCoords(1.25, -1.75), Math.toRadians(45)), Math.toRadians(0));


        waitForStart();

        if (isStopRequested()) return;

        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToCenter, setWristToBasket, setWristToFloorPickup,
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw,
        // moveClawToTopBasket, moveClawToTopRung, moveClawToHumanPickup, resetArm,

        Actions.runBlocking(
                new SequentialAction(
                        armClaw.RunMethod("openClaw"),
                        armClaw.RunMethod("moveClawToTopRung", 1),
                        DriveToRungs.build(),
                        armClaw.Wait(5),
                        armClaw.RunMethod("closeClaw"),
                        armClaw.RunMethod("resetArm", 2),
                        DriveToFirstSample.build(),
                        armClaw.waitUntilFinishedMoving()
                )
        );
        telemetry.addLine("Finished Auton");

    }

}
