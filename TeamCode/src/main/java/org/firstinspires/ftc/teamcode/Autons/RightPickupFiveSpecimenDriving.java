package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PedroTrajectoryActionBuilder;


@Autonomous(name="RightPickupFiveSpecimenDriving")
public class RightPickupFiveSpecimenDriving extends CommandOpMode {

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
                .strafeToConstantHeading(tileCoords(0.33, -1.33))
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart1 = drive.actionBuilder(DriveToChamber1.endPose())
                .bezierToConstantHeading(tileCoords(1.7, -2.3), tileCoords(1.25, -0.8), tileCoords(2.15, -0.5))
                .setPathEndTimeoutConstraint(50)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart2 = drive.actionBuilder(PushPresetSamplesPart1.endPose())
                .strafeToConstantHeading(tileCoords(2.15, -2.2))
                .setZeroPowerAccelerationMultiplier(4)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart3 = drive.actionBuilder(PushPresetSamplesPart2.endPose())
                .strafeToConstantHeading(tileCoords(2.05, -1.3))
                .splineToConstantHeading(tileCoords(2.6, -0.6), Math.toRadians(0))
                //.setPathEndTimeoutConstraint(50)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart4 = drive.actionBuilder(PushPresetSamplesPart3.endPose())
                .strafeToConstantHeading(tileCoords(2.6, -2.2))
                .setZeroPowerAccelerationMultiplier(4)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart5 = drive.actionBuilder(PushPresetSamplesPart4.endPose())
                .strafeToConstantHeading(tileCoords(2.6, -1.5))
                .splineToConstantHeading(tileCoords(2.83, -0.7), Math.toRadians(0))
                //.setPathEndTimeoutConstraint(50)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder PushPresetSamplesPart6 = drive.actionBuilder(PushPresetSamplesPart5.endPose())
                .strafeToConstantHeading(tileCoords(2.84, -2.8))
                .endAfterTimeout(5.0);

        /*
        PedroTrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(PushPresetSamplesPart6.endPose())
                .strafeToLinearHeading(tileCoords(2.67, -2.8), Math.toRadians(90))
                //.strafeToConstantHeading(tileCoords(2.1, -2.72))
                .setPathEndTimeoutConstraint(100)
                .endAfterTimeout(3.0);

        PedroTrajectoryActionBuilder DriveToHumanPlayer1half = drive.actionBuilder(PushPresetSamplesPart6.endPose())
                .strafeToConstantHeading(tileCoords(2.1, -2.72))
                .setPathEndTimeoutConstraint(250);

         */

        PedroTrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(PushPresetSamplesPart6.endPose())
                //.strafeToConstantHeading(tileCoords(0.28, -1.33))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.25, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(0)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(DriveToChamber2.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .setPathEndTimeoutConstraint(100)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(DriveToHumanPlayer2.endPose())
                //.strafeToConstantHeading(tileCoords(0.18, -1.33))
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(tileCoords(0.12, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(0)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToHumanPlayer3 = drive.actionBuilder(DriveToChamber3.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .setPathEndTimeoutConstraint(100)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToChamber4 = drive.actionBuilder(DriveToHumanPlayer3.endPose())
                //.strafeToConstantHeading(tileCoords(0.11, -1.33))
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(tileCoords(0.01, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(0)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToHumanPlayer4 = drive.actionBuilder(DriveToChamber4.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.2), Math.toRadians(360-55))
                .strafeToConstantHeading(tileCoords(1.8, -2.8))
                .setPathEndTimeoutConstraint(100)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToChamber5 = drive.actionBuilder(DriveToHumanPlayer4.endPose())
                //.strafeToConstantHeading(tileCoords(0.05, -1.33))
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(tileCoords(-0.12, -1.33), Math.toRadians(110))
                .setPathEndTimeoutConstraint(0)
                .endAfterTimeout(5.0);

        PedroTrajectoryActionBuilder DriveToPark = drive.actionBuilder(DriveToChamber5.endPose())
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(2.1, -2.55), Math.toRadians(360-20));


        SequentialAction PlaceSpecimen1 = new SequentialAction(
                armSystem.Wait(0.3),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.1),
                armSystem.RunMethod("stopClaw", 0.4)
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.45),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.1),
                armSystem.RunMethod("stopClaw", 0.4)
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.45),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.1),
                armSystem.RunMethod("stopClaw", 0.4)
        );

        SequentialAction PlaceSpecimen4 = new SequentialAction(
                armSystem.Wait(0.45),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.1),
                armSystem.RunMethod("stopClaw", 0.4)
        );

        SequentialAction PlaceSpecimen5 = new SequentialAction(
                armSystem.Wait(0.45),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.2),
                armSystem.RunMethod("stopClaw", 0.4)
        );


        // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

        // ArmClaw method names:
        // openClaw, closeClaw, slowCloseClaw, reallySlowCloseClaw, stopClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, setWristToRaisedFloor, depositSpecimen
        // dropSamplePickup, setWristToAutoAngle
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot


        drive.update();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("closeClaw"),
                        armSystem.RunMethod("setWristToAutoAngle"),
                        armSystem.RunMethod("moveArmDirectly", 0.0, 76.0, 210.0),
                        armSystem.RunMethod("setWristToAutoAngle", 0.5),
                        armSystem.Wait(0.2),
                        DriveToChamber1.build(),

                        armSystem.RunMethod("moveClawToTopRungAdjusted", 0.0, -0.5+5, 10.0),
                        PlaceSpecimen1,
                        armSystem.RunMethod("resetArm", 0.3),
                        PushPresetSamplesPart1.build(),
                        PushPresetSamplesPart2.build(),
                        PushPresetSamplesPart3.build(),
                        PushPresetSamplesPart4.build(),
                        PushPresetSamplesPart5.build(),
                        armSystem.RunMethod("moveClawToHumanPickup", 0.3),
                        armSystem.RunMethod("closeClaw", 0.5),
                        PushPresetSamplesPart6.build(),

                        // DriveToHumanPlayer1half.build(),

                        // armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToAutoAngle"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber2.build(),

                        armSystem.RunMethod("moveClawToTopRungAuto"),
                        PlaceSpecimen2,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.3),
                        armSystem.RunMethod("closeClaw", 0.8),
                        DriveToHumanPlayer2.build(),

                        // armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToAutoAngle"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber3.build(),

                        armSystem.RunMethod("moveClawToTopRungAuto"),
                        PlaceSpecimen3,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.3),
                        armSystem.RunMethod("closeClaw", 0.8),
                        DriveToHumanPlayer3.build(),

                        // armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToAutoAngle"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber4.build(),

                        armSystem.RunMethod("moveClawToTopRungAuto"),
                        PlaceSpecimen4,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.3),
                        armSystem.RunMethod("slowCloseClaw", 0.8),
                        DriveToHumanPlayer4.build(),

                        // armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToAutoAngle"),
                        armSystem.RunMethod("moveArmDirectly", 0.2, 76.0, 195.0),
                        DriveToChamber5.build(),

                        armSystem.RunMethod("moveClawToTopRungAuto"),
                        PlaceSpecimen5,
                        armSystem.RunMethod("resetArm", 0.3),
                        DriveToPark.build(),
                        armSystem.Wait(0.5)
                ));

        SubsystemData.LocalizationCoordsAligned = new boolean[]{true, true};

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}