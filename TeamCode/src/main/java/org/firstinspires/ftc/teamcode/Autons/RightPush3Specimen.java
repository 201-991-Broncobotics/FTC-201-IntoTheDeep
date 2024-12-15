package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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

import java.util.Arrays;


@Autonomous(name="RightPush3Specimen")
public class RightPush3Specimen extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 4.0), Math.toRadians(90));
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        drive.resetSwerveWheelAngles(); // reset swerve wheels

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        armSystem.resetAndPrepareArm(); // set arm into start position and align zeros


        // setup roadrunner trajectories
        TrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.25, -1.5))
                .strafeToConstantHeading(tileCoords(0.25, -1.25));

        VelConstraint VelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        VelConstraint VelConstraint2 = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(35.0),
                new AngularVelConstraint(Math.PI / 2)
        ));


        TrajectoryActionBuilder PushPresetSamples = drive.actionBuilder(new Pose2d(tileCoords(0.25, -1.25), startPose.heading.toDouble())) // DriveToRungs.fresh()
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(1, -1.7), Math.toRadians(0))
                .splineToConstantHeading(tileCoords(1.55, -0.7), Math.toRadians(70), VelConstraint)
                .splineToConstantHeading(tileCoords(2.07, -0.5), Math.toRadians(0))
                .strafeToConstantHeading(tileCoords(2.07, -2.25))
                .strafeToConstantHeading(tileCoords(2.07, -0.8), VelConstraint2)
                .splineToConstantHeading(tileCoords(2.6, -0.6), Math.toRadians(0), VelConstraint2)
                .strafeToConstantHeading(tileCoords(2.6, -2.5))
                .strafeToConstantHeading(tileCoords(2.6, -2.25), VelConstraint2);

        TrajectoryActionBuilder DriveToHumanPlayer1 = drive.actionBuilder(new Pose2d(tileCoords(2.6, -2.25), startPose.heading.toDouble()))
                .strafeToConstantHeading(tileCoords(2, -2.4))
                .strafeToConstantHeading(tileCoords(2, -2.85));

        TrajectoryActionBuilder DriveToChamber2 = drive.actionBuilder(new Pose2d(tileCoords(2, -2.85), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.18, -1.3), Math.toRadians(90), VelConstraint2);

        TrajectoryActionBuilder DriveToHumanPlayer2 = drive.actionBuilder(new Pose2d(tileCoords(0.15, -1.3), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(2, -2.4), Math.toRadians(360-35))
                .strafeToConstantHeading(tileCoords(2, -2.85));

        TrajectoryActionBuilder DriveToChamber3 = drive.actionBuilder(new Pose2d(tileCoords(2, -2.85), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.1, -1.3), Math.toRadians(90), VelConstraint2);

        TrajectoryActionBuilder DriveToPark = drive.actionBuilder(new Pose2d(tileCoords(0.1, -1.3), startPose.heading.toDouble()))
                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(2.1, -2.5), Math.toRadians(360-20));


        SequentialAction PlaceSpecimen1 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.3),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen2 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.3),
                armSystem.RunMethod("openClaw")
        );

        SequentialAction PlaceSpecimen3 = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.3),
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
                        armSystem.Wait(0.2),
                        armSystem.RunMethod("moveArmDirectly", 0.0, 76.0, 205.0),
                        armSystem.Wait(0.2),
                        DriveToChamber1.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen1,
                        armSystem.RunMethod("resetArm", 0.3),
                        PushPresetSamples.build(),
                        armSystem.RunMethod("moveClawToHumanPickup"),
                        DriveToHumanPlayer1.build(),
                        armSystem.Wait(0.2),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 76.0, 205.0),
                        DriveToChamber2.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        armSystem.Wait(0.2),
                        PlaceSpecimen2,
                        armSystem.RunMethod("moveClawToHumanPickup", 0.5),
                        DriveToHumanPlayer2.build(),
                        armSystem.Wait(0.2),
                        armSystem.RunMethod("closeClaw"),
                        armSystem.Wait(0.4),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.RunMethod("moveArmDirectly", 0.5, 76.0, 205.0),
                        DriveToChamber3.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        armSystem.Wait(0.2),
                        PlaceSpecimen3,
                        armSystem.RunMethod("resetArm", 0.3),
                        DriveToPark.build(),
                        armSystem.Wait(0.5)
        ));

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
