package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;


@Autonomous(name="RightSampleCollection", group="Red Side")
public class RightSampleCollection extends CommandOpMode {

    @Override
    public void initialize() {

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(functions.tiles(0.5), functions.tiles(-3) + 7.09), Math.toRadians(90));
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);

        ArmSystem armClaw = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // always running
        drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armClaw.setDefaultCommand(new ArmClawAutonCommand(armClaw));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToRungs = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.2, -1.6));

        TrajectoryActionBuilder DriveToFirstSample = DriveToRungs.fresh()
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(tileCoords(1.4, -1.6), Math.toRadians(45)), Math.toRadians(0));

        TrajectoryActionBuilder PointTowardObservation = DriveToFirstSample.fresh()
                .turnTo(Math.toRadians(-45));



        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToCenter, setWristToBasket, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw,
        // moveClawToTopBasket, moveClawToTopRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, moveArmDirectly, setWrist

        // Actions:
        // Wait, RunMethod ^, waitUntilFinishedAwaiting, waitUntilFinishedMoving

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armClaw.RunMethod("closeClaw"),
                        armClaw.RunMethod("moveClawToTopRung"),
                        DriveToRungs.build(),
                        armClaw.waitUntilFinishedMoving(5),
                        armClaw.Wait(1.5),
                        armClaw.RunMethod("depositSpecimen"),
                        armClaw.Wait(1.5),
                        armClaw.RunMethod("resetArm"),
                        armClaw.RunMethod("moveArmToPoint", 2, new Vector2d(384.8, 10)), // Distance + PivotAxleOffset - RetractedExtensionLength
                        armClaw.RunMethod("setWristToFloorPickup", 2.5),
                        DriveToFirstSample.build(),
                        armClaw.waitUntilFinishedMoving(5),
                        armClaw.RunMethod("closeClaw"),
                        armClaw.Wait(1),
                        armClaw.RunMethod("resetArm"),
                        armClaw.RunMethod("moveArmToPoint", 1.5, new Vector2d(696, 40)),
                        PointTowardObservation.build(),
                        armClaw.waitUntilFinishedAwaiting(),
                        armClaw.waitUntilFinishedMoving(5),
                        armClaw.RunMethod("openClaw")
                )
        );

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
