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
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

import com.arcrobotics.ftclib.command.RunCommand;


@Autonomous(name="RightPushThreeSpecimen")
public class RightPushThreeSpecimen extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 4.0), Math.toRadians(90));
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


        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.2, -1.5))
                .strafeToConstantHeading(tileCoords(0.2, -1.3));

        TrajectoryActionBuilder PushPresetSamples = drive.actionBuilder(new Pose2d(tileCoords(0.2, -1.3), startPose.heading.toDouble())) // DriveToRungs.fresh()
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(1, -1.8), Math.toRadians(0))
                .splineToConstantHeading(tileCoords(1.55, -0.6), Math.toRadians(60))
                .splineToConstantHeading(tileCoords(2.1, -0.4), Math.toRadians(0));

        TrajectoryActionBuilder PushPresetSamples2 = drive.actionBuilder(new Pose2d(tileCoords(2.1, -0.4), startPose.heading.toDouble())) // DriveToRungs.fresh()
                .strafeToConstantHeading(tileCoords(2.1, -2.3))
                .strafeToConstantHeading(tileCoords(2.1, -0.4))
                .splineToConstantHeading(tileCoords(2.5, -0.6), Math.toRadians(180))
                .strafeToConstantHeading(tileCoords(2.5, -2.3));

        TrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(new Pose2d(tileCoords(2.5, -2.3), startPose.heading.toDouble()))
                .strafeToConstantHeading(tileCoords(2, -2.4))
                .strafeToConstantHeading(tileCoords(2, -2.85));

        TrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(new Pose2d(tileCoords(2, -2.85), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.1, -1.3), Math.toRadians(90));

        TrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(new Pose2d(tileCoords(0.1, -1.3), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(2, -2.4), Math.toRadians(360-35))
                .strafeToConstantHeading(tileCoords(2, -2.85));

        TrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(new Pose2d(tileCoords(2, -2.85), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0, -1.3), Math.toRadians(90));


        SequentialAction PlaceSpecimen = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("openClaw")
        );



        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw, dropSamplePickup
        // moveClawToTopBasket, moveClawToTopRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, moveArmDirectly, setWrist

        drive.updatePoseEstimate();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("closeClaw"),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.Wait(0.5),
                        armSystem.RunMethod("moveArmDirectly", 0.01, 79.0, 215.0),
                        armSystem.Wait(0.3),
                        DriveToChamber1.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        armSystem.Wait(0.25),
                        PlaceSpecimen,
                        armSystem.RunMethod("resetArm", 0.3),
                        PushPresetSamples.build(),
                        armSystem.Wait(1),
                        PushPresetSamples2.build(),
                        armSystem.RunMethod("moveClawToHumanPickup"),
                        DriveToHumanPlayer1.build(),
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
                        armSystem.RunMethod("resetArm", 0.3),
                        drive.actionBuilder(new Pose2d(tileCoords(0.1, -1.1), startPose.heading.toDouble()))
                                .strafeToConstantHeading(tileCoords(0.1, -2)).build(),
                        armSystem.Wait(0.5)
        ));

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
