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


@Autonomous(name="RightPushFourSpecimen")
public class RightPushFourSpecimen extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 4.0), Math.toRadians(90));
        Follower drive = new Follower(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        drive.getRRDrive().resetSwerveWheelAngles(); // reset swerve wheels

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        armSystem.resetAndPrepareArm(); // set arm into start position and align zeros


        // setup roadrunner trajectories
        PedroTrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.25, -1.3));

        PedroTrajectoryActionBuilder PushPresetSamplesPart1 = drive.actionBuilder(DriveToChamber1.endPose())
                .bezierToConstantHeading(tileCoords(1.95, -2.5), tileCoords(1.05, -0.7), tileCoords(2.1, -0.5))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder PushPresetSamplesPart2 = drive.actionBuilder(PushPresetSamplesPart1.endPose())
                .strafeToConstantHeading(tileCoords(2.05, -2.3))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder PushPresetSamplesPart3 = drive.actionBuilder(PushPresetSamplesPart2.endPose())
                .strafeToConstantHeading(tileCoords(2.05, -1.1))
                .splineToConstantHeading(tileCoords(2.5, -0.6), Math.toRadians(0))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder PushPresetSamplesPart4 = drive.actionBuilder(PushPresetSamplesPart3.endPose())
                .strafeToConstantHeading(tileCoords(2.70, -2.3))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder PushPresetSamplesPart5 = drive.actionBuilder(PushPresetSamplesPart4.endPose())
                .strafeToConstantHeading(tileCoords(2.65, -1.1))
                .splineToConstantHeading(tileCoords(2.85, -0.6), Math.toRadians(0))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder PushPresetSamplesPart6 = drive.actionBuilder(PushPresetSamplesPart5.endPose())
                .strafeToConstantHeading(tileCoords(2.85, -2.4))
                .setPathEndTimeoutConstraint(50);

        PedroTrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(PushPresetSamplesPart6.endPose())
                .strafeToConstantHeading(tileCoords(2.4, -2.3))
                .strafeToConstantHeading(tileCoords(2, -2.73));

        PedroTrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(DriveToHumanPlayer1.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.22, -1.3), Math.toRadians(90));

        PedroTrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(DriveToChamber2.endPose())
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(1.5, -2.3), Math.toRadians(360-45))
                .strafeToConstantHeading(tileCoords(1.8, -2.73));

        PedroTrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(DriveToHumanPlayer2.endPose())
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.15, -1.3), Math.toRadians(90));

        PedroTrajectoryActionBuilder DriveToPark = drive.actionBuilder(DriveToChamber3.endPose())
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(2.1, -2.5), Math.toRadians(360-20));


        SequentialAction PlaceSpecimen1 = new SequentialAction(
                armSystem.Wait(0.25),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.25),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.25),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.25),
                armSystem.RunMethod("openClaw")
        );



        // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw, dropSamplePickup
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot

        drive.update();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("closeClaw"),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.Wait(0.2),
                        armSystem.RunMethod("moveArmDirectly", 0.0, 76.0, 180.0),
                        // armSystem.Wait(0.3),
                        DriveToChamber1.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen1,
                        armSystem.RunMethod("resetArm", 0.3),
                        PushPresetSamplesPart1.build(),
                        PushPresetSamplesPart2.build(),
                        PushPresetSamplesPart3.build(),
                        PushPresetSamplesPart4.build(),
                        PushPresetSamplesPart5.build(),
                        PushPresetSamplesPart6.build(),
                        armSystem.RunMethod("moveClawToHumanPickup"),
                        DriveToHumanPlayer1.build(),
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 76.0, 180.0),
                        DriveToChamber2.build(),
                        //armSystem.Wait(0.4),
                        armSystem.RunMethod("moveClawToTopRung"),
                        //armSystem.Wait(0.2),
                        PlaceSpecimen2,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.4),
                        armSystem.Wait(0.1),
                        DriveToHumanPlayer2.build(),
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 76.0, 180.0),
                        DriveToChamber3.build(),
                        //armSystem.Wait(0.4),
                        armSystem.RunMethod("moveClawToTopRung"),
                        //armSystem.Wait(0.2),
                        PlaceSpecimen3,
                        armSystem.RunMethod("resetArm", 0.3),
                        DriveToPark.build(),
                        armSystem.Wait(0.5)
        ));

        SubsystemData.eligibleForAutoDriving = true;

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
