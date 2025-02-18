package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PedroTrajectoryActionBuilder;

import com.arcrobotics.ftclib.command.RunCommand;


@Autonomous(name="RightPickupFiveSpecimen")
public class RightPickupFiveSpecimen extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 2.0), Math.toRadians(90));
        Follower drive = new Follower(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // drive.getRRDrive().resetSwerveWheelAngles(); // reset swerve wheels

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        armSystem.prepareServosForAuton();

        armSystem.resetAndPrepareArm(); // set arm into start position and align zeros


        // setup roadrunner trajectories
        PedroTrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.25, -1.33));

        PedroTrajectoryActionBuilder DriveToSample1 = drive.actionBuilder(DriveToChamber1.endPose())
                .setTangent(Math.toRadians(360-45))
                .splineToConstantHeading(tileCoords(2.15, -1.8), Math.toRadians(0));

        PedroTrajectoryActionBuilder DriveToDropOffSample1 = drive.actionBuilder(DriveToSample1.endPose())
                .strafeToConstantHeading(tileCoords(2.15, -2.3));

        PedroTrajectoryActionBuilder DriveToSample2 = drive.actionBuilder(DriveToDropOffSample1.endPose())
                .strafeToConstantHeading(tileCoords(2.65, -1.8));

        PedroTrajectoryActionBuilder DriveToDropOffSample2 = drive.actionBuilder(DriveToSample2.endPose())
                .strafeToConstantHeading(tileCoords(2.65, -2.3));

        PedroTrajectoryActionBuilder DriveToSample3 = drive.actionBuilder(DriveToDropOffSample2.endPose())
                .strafeToLinearHeading(tileCoords(2.5, -1.65), Math.toRadians(45));

        PedroTrajectoryActionBuilder DriveToDropOffSample3 = drive.actionBuilder(DriveToSample3.endPose())
                .strafeToLinearHeading(tileCoords(2.5, -2.3), Math.toRadians(90));

        PedroTrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(DriveToDropOffSample3.endPose())
                .strafeToConstantHeading(tileCoords(2.1, -2.4))
                .setPathEndTimeoutConstraint(200);

        PedroTrajectoryActionBuilder DriveToHumanPlayer1half = drive.actionBuilder(DriveToHumanPlayer1.endPose())
                .strafeToConstantHeading(tileCoords(2, -2.715))
                .endAfterTimeout(7.0);

        PedroTrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(DriveToHumanPlayer1.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.21, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(200);

        PedroTrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(DriveToChamber2.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .endAfterTimeout(7.0);

        PedroTrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(DriveToHumanPlayer2.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.11, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(200);

        PedroTrajectoryActionBuilder DriveToHumanPlayer3 = drive.actionBuilder(DriveToChamber3.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .endAfterTimeout(7.0);

        PedroTrajectoryActionBuilder DriveToChamber4 = drive.actionBuilder(DriveToHumanPlayer3.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.07, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(200);

        PedroTrajectoryActionBuilder DriveToHumanPlayer4 = drive.actionBuilder(DriveToChamber4.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .endAfterTimeout(7.0);

        PedroTrajectoryActionBuilder DriveToChamber5 = drive.actionBuilder(DriveToHumanPlayer4.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.0, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(200);

        PedroTrajectoryActionBuilder DriveToPark = drive.actionBuilder(DriveToChamber5.endPose())
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(2.1, -2.5), Math.toRadians(360-20));


        SequentialAction PlaceSpecimen1 = new SequentialAction(
                armSystem.Wait(0.5),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw"),
                armSystem.RunMethod("stopClaw", 0.5)
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.2),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw"),
                armSystem.RunMethod("stopClaw", 0.5)
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.2),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw"),
                armSystem.RunMethod("stopClaw", 0.5)
        );

        SequentialAction PlaceSpecimen4 = new SequentialAction(
                armSystem.Wait(0.2),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw"),
                armSystem.RunMethod("stopClaw", 0.5)
        );


        // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

        // ArmClaw method names:
        // openClaw, closeClaw, slowCloseClaw, reallySlowCloseClaw, stopClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, setWristToRaisedFloor, depositSpecimen
        // dropSamplePickup
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot


        drive.update();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("slowCloseClaw"),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.0, 76.0, 180.0),
                        armSystem.RunMethod("setWristToStraight", 0.5),
                        armSystem.Wait(0.2),
                        DriveToChamber1.build(),
                        armSystem.RunMethod("moveClawToTopRungAuto", 0.0, 0.0, -20.0),
                        PlaceSpecimen1,
                        armSystem.RunMethod("resetArm", 0.3),
                        armSystem.RunMethod("setWristToStraight", 0.5),
                        armSystem.RunMethod("moveArmDirectly", 0.6, 0.0, 70.0),
                        armSystem.RunMethod("closeClaw", 0.7),
                        DriveToSample1.build(),

                        armSystem.Wait(0.3),
                        armSystem.RunMethod("setWristToFloorPickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("slowCloseClaw"),
                        armSystem.RunMethod("moveClawToHumanPickup", 0.2),
                        DriveToDropOffSample1.build(),

                        armSystem.RunMethod("openClaw"),
                        armSystem.Wait(0.3),
                        armSystem.RunMethod("stopClaw"),
                        armSystem.RunMethod("resetArm", 0.1),
                        armSystem.RunMethod("setWristToStraight", 0.5),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 0.0, 80.0),
                        armSystem.RunMethod("closeClaw", 0.5),
                        DriveToSample2.build(),

                        armSystem.Wait(0.3),
                        armSystem.RunMethod("setWristToFloorPickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("slowCloseClaw"),
                        armSystem.RunMethod("moveClawToHumanPickup", 0.2),
                        DriveToDropOffSample2.build(),

                        armSystem.RunMethod("openClaw"),
                        armSystem.Wait(0.3),
                        armSystem.RunMethod("stopClaw"),
                        armSystem.RunMethod("resetArm", 0.1),
                        armSystem.RunMethod("setWristToStraight", 0.5),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 0.0, 130.0),
                        armSystem.RunMethod("closeClaw", 0.5),
                        DriveToSample3.build(),

                        armSystem.Wait(0.3),
                        armSystem.RunMethod("setWristToFloorPickup"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("slowCloseClaw"),
                        armSystem.RunMethod("moveClawToHumanPickup", 0.2),
                        DriveToDropOffSample3.build(),

                        armSystem.RunMethod("openClaw"),
                        armSystem.Wait(0.3),
                        DriveToHumanPlayer1.build(),

                        armSystem.RunMethod("closeClaw"),
                        DriveToHumanPlayer1half.build(),

                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber2.build(),

                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen2,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.4),
                        armSystem.RunMethod("closeClaw", 0.8),
                        DriveToHumanPlayer2.build(),

                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber3.build(),

                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen3,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.4),
                        armSystem.RunMethod("closeClaw", 0.8),
                        DriveToHumanPlayer3.build(),

                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber4.build(),

                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen3,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.4),
                        armSystem.RunMethod("slowCloseClaw", 0.8),
                        DriveToHumanPlayer4.build(),

                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber5.build(),

                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen4,
                        armSystem.RunMethod("resetArm", 0.3),
                        DriveToPark.build(),
                        armSystem.Wait(0.5)
                ));

        SubsystemData.LocalizationCoordsAligned = new boolean[]{true, true};

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}