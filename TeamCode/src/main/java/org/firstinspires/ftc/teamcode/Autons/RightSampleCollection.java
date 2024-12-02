package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;


@Autonomous(name="RightSampleCollection")
public class RightSampleCollection extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 7.09), Math.toRadians(90));
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToRungs = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.2, -1.1));

        TrajectoryActionBuilder DriveToFirstSample = drive.actionBuilder(new Pose2d(tileCoords(0.2, -1.6), startPose.heading.toDouble())) // DriveToRungs.fresh()
                .setTangent(Math.toRadians(0))
                .lineToX(tiles(1.2))
                .splineToLinearHeading(new Pose2d(tileCoords(1.5, -0.5), Math.toRadians(0)), Math.toRadians(90))
                .strafeToConstantHeading(tileCoords(2.0, -0.5))
                .strafeToConstantHeading(tileCoords(2.0, -2.5));

        TrajectoryActionBuilder AlignToSample = drive.actionBuilder(new Pose2d(tileCoords(1.4, -1.6), 90))
                .turnTo(Math.toRadians(45));

        TrajectoryActionBuilder PointTowardObservation = drive.actionBuilder(new Pose2d(tileCoords(1.4, -1.6), Math.toRadians(45))) // DriveToFirstSample.fresh()
                .turnTo(Math.toRadians(360-90));



        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToCenter, setWristToBasket, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw,
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, moveArmDirectly, setWrist

        // Actions:
        // Wait, RunMethod ^, waitUntilFinishedAwaiting, waitUntilFinishedMoving

        drive.updatePoseEstimate();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        //armSystem.RunMethod("closeClaw"),
                        //armSystem.RunMethod("moveClawToRamRung"),
                        DriveToRungs.build(),
                        armSystem.Wait(1),
                        //armSystem.RunMethod("openClaw"),
                        //armSystem.RunMethod("resetArm", 0.75),
                        armSystem.Wait(0.5),
                        DriveToFirstSample.build()
                )
        );

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
