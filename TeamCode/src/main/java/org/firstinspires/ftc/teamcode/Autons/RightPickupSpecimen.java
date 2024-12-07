package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.RunCommand;

import java.util.Arrays;


@Autonomous(name="RightPickupSpecimen")
public class RightPickupSpecimen extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 2.0), Math.toRadians(90));
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        armSystem.resetAndPrepareArm(); // set arm into start position and align zeros


        VelConstraint MaxSpeedVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI)
        ));

        VelConstraint SlowDownVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint SlowDownAccelConstraint = new ProfileAccelConstraint(-30.0, 30.0);





        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.2, -1.5))
                .strafeToConstantHeading(tileCoords(0.2, -1.3));

        TrajectoryActionBuilder DriveToSample1 = drive.actionBuilder(new Pose2d(tileCoords(0.2, -1.3), startPose.heading.toDouble())) // DriveToRungs.fresh()
                .setTangent(Math.toRadians(360-60))
                .splineToLinearHeading(new Pose2d(tileCoords(1.4, -1.7), Math.toRadians(45)), Math.toRadians(0));

        TrajectoryActionBuilder Turn1 = drive.actionBuilder(new Pose2d(tileCoords(1.4, -1.7), Math.toRadians(45)))
                .turnTo(Math.toRadians(360-45));

        TrajectoryActionBuilder DriveToSample2 = drive.actionBuilder(new Pose2d(tileCoords(1.4, -1.7), Math.toRadians(360-45)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(tileCoords(1.8, -1.7), Math.toRadians(45)), Math.toRadians(0), SlowDownVelConstraint, SlowDownAccelConstraint);

        TrajectoryActionBuilder Turn2 = drive.actionBuilder(new Pose2d(tileCoords(1.8, -1.7), Math.toRadians(45)))
                .turnTo(Math.toRadians(360-60));

        TrajectoryActionBuilder DriveToSample3 = drive.actionBuilder(new Pose2d(tileCoords(1.8, -1.7), Math.toRadians(360-60)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(tileCoords(2.2, -1.7), Math.toRadians(45)), Math.toRadians(0), SlowDownVelConstraint, SlowDownAccelConstraint);

        TrajectoryActionBuilder Turn3 = drive.actionBuilder(new Pose2d(tileCoords(2.2, -1.7), Math.toRadians(45)))
                .turnTo(Math.toRadians(360-70));

        TrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(new Pose2d(tileCoords(2.2, -1.7), Math.toRadians(360-70)))
                .strafeToLinearHeading(tileCoords(1.8, -2.4), Math.toRadians(90))
                .strafeToConstantHeading(tileCoords(1.8, -2.8));

        TrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(new Pose2d(tileCoords(1.8, -2.8), Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.15, -1.25), Math.toRadians(125), MaxSpeedVelConstraint);

        TrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(new Pose2d(tileCoords(0.15, -1.25), Math.toRadians(90)))
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.8, -2.8), Math.toRadians(270), MaxSpeedVelConstraint);

        TrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(new Pose2d(tileCoords(1.8, -2.8), Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.1, -1.25), Math.toRadians(125), MaxSpeedVelConstraint);

        TrajectoryActionBuilder DriveToHumanPlayer3 = drive.actionBuilder(new Pose2d(tileCoords(0.1, -1.25), Math.toRadians(90)))
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.8, -2.8), Math.toRadians(270), MaxSpeedVelConstraint);

        TrajectoryActionBuilder DriveToChamber4 = drive.actionBuilder(new Pose2d(tileCoords(1.8, -2.8), Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(-0.05, -1.25), Math.toRadians(125), MaxSpeedVelConstraint);




        SequentialAction PlaceSpecimen1 = new SequentialAction(
                armSystem.Wait(0.5),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.75),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.5),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.75),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen4 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("openClaw")
        );



        // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw, dropSamplePickup
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot

        drive.updatePoseEstimate();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("closeClaw"),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("moveClawToTopRung"),
                        DriveToChamber1.build(),
                        PlaceSpecimen1,
                        armSystem.RunMethod("resetArm", 0.1),
                        armSystem.RunMethod("holdClawAtFieldCoordinate", 1, tileCoords(2.1, -1.1), Constants.pivotAxleHeight + 30.0),

                        DriveToSample1.build(),
                        armSystem.waitUntilFinishedMoving(2),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("dropSamplePickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("setExtension", 0, 200.0),
                        armSystem.RunMethod("setExtension", 1, 600.0),

                        Turn1.build(),
                        armSystem.RunMethod("openClaw"),
                        armSystem.RunMethod("setExtension", 0, 200.0),
                        armSystem.RunMethod("holdClawAtFieldCoordinate", 1, tileCoords(2.5, -1.1), Constants.pivotAxleHeight + 30.0),

                        DriveToSample2.build(),
                        armSystem.waitUntilFinishedMoving(2),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("dropSamplePickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("setExtension", 0, 200.0),
                        armSystem.RunMethod("setExtension", 1, 600.0),

                        Turn2.build(),
                        armSystem.RunMethod("openClaw"),
                        armSystem.RunMethod("setExtension", 0, 200.0),
                        armSystem.RunMethod("holdClawAtFieldCoordinate", 1, tileCoords(2.9, -1.1), Constants.pivotAxleHeight + 30.0),

                        DriveToSample3.build(),
                        armSystem.waitUntilFinishedMoving(2),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("dropSamplePickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("setExtension", 0, 200.0),
                        armSystem.RunMethod("setExtension", 1, 600.0),

                        Turn3.build(),
                        armSystem.RunMethod("openClaw"),
                        armSystem.RunMethod("moveClawToHumanPickup", 0.1),

                        DriveToHumanPlayer1.build(),
                        armSystem.waitUntilFinishedMoving(2),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("moveClawToTopRung", 0.5),

                        DriveToChamber2.build(),
                        PlaceSpecimen2,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.5),

                        DriveToHumanPlayer2.build(),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("moveClawToTopRung", 0.5),

                        DriveToChamber3.build(),
                        PlaceSpecimen3,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.5),

                        DriveToHumanPlayer3.build(),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("moveClawToTopRung", 0.5),

                        DriveToChamber4.build(),
                        PlaceSpecimen4,
                        armSystem.RunMethod("resetArm", 0.5),
                        armSystem.waitUntilFinishedMoving(2),

                        armSystem.Wait(0.5)
                ));

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}