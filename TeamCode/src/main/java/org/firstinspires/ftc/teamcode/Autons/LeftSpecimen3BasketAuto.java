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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PedroTrajectoryActionBuilder;

import com.arcrobotics.ftclib.command.RunCommand;

import java.util.Arrays;


@Autonomous(name="LeftSpecimen3BasketAuto")
public class LeftSpecimen3BasketAuto extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(-0.5), tiles(-3) + 7.09), Math.toRadians(90));
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


        // setup roadrunner trajectories
        PedroTrajectoryActionBuilder DriveToChamber1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(-0.2, -1.5))
                .strafeToConstantHeading(tileCoords(-0.2, -1.2));

        PedroTrajectoryActionBuilder DriveToSample1 = drive.actionBuilder(new Pose2d(tileCoords(-0.2, -1.2), Math.toRadians(90)))
                .setTangent(Math.toRadians(180+45))
                .splineToLinearHeading(new Pose2d(tileCoords(-1.5, -1.6), Math.toRadians(135)), Math.toRadians(180));

        PedroTrajectoryActionBuilder DriveToBasket1 = drive.actionBuilder(new Pose2d(tileCoords(-1.5, -1.6), Math.toRadians(135)))
                .strafeToLinearHeading(tileCoords(-2.45, -2.4), Math.toRadians(225));

        PedroTrajectoryActionBuilder DriveToSample2 = drive.actionBuilder(new Pose2d(tileCoords(-2.5, -2.5), Math.toRadians(225)))
                .strafeToLinearHeading(tileCoords(-1.95, -1.6), Math.toRadians(135));

        PedroTrajectoryActionBuilder DriveToBasket2 = drive.actionBuilder(new Pose2d(tileCoords(-1.95, -1.6), Math.toRadians(135)))
                .strafeToLinearHeading(tileCoords(-2.45, -2.4), Math.toRadians(225));

        PedroTrajectoryActionBuilder DriveToSample3 = drive.actionBuilder(new Pose2d(tileCoords(-2.5, -2.5), Math.toRadians(225)))
                .strafeToLinearHeading(tileCoords(-2.4, -1.6), Math.toRadians(135));

        PedroTrajectoryActionBuilder DriveToBasket3 = drive.actionBuilder(new Pose2d(tileCoords(-2.4, -1.6), Math.toRadians(135)))
                .strafeToLinearHeading(tileCoords(-2.45, -2.4), Math.toRadians(225));

        PedroTrajectoryActionBuilder ParkAtRungs = drive.actionBuilder(new Pose2d(tileCoords(-2.5, -2.5), Math.toRadians(225)))
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(tileCoords(-1, -0.5), Math.toRadians(0)), Math.toRadians(0));





        SequentialAction PlaceSpecimen = new SequentialAction(
                armSystem.Wait(0.4),
                armSystem.RunMethod("depositSpecimen"),
                armSystem.Wait(0.3),
                armSystem.RunMethod("openClaw")
        );


        SequentialAction DumpSampleInBasket1 = new SequentialAction(
                armSystem.RunMethod("moveClawToTopBasket"), // a second time in order for the arm to actually push for max extension
                armSystem.Wait(0.75),
                armSystem.RunMethod("openClaw"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("setWrist", 0, 90.0),
                armSystem.Wait(0.5)
        );
        SequentialAction DumpSampleInBasket2 = new SequentialAction(
                armSystem.RunMethod("moveClawToTopBasket"), // a second time in order for the arm to actually push for max extension
                armSystem.Wait(0.75),
                armSystem.RunMethod("openClaw"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("setWrist", 0, 90.0),
                armSystem.Wait(0.5)
        );
        SequentialAction DumpSampleInBasket3 = new SequentialAction(
                armSystem.RunMethod("moveClawToTopBasket"), // a second time in order for the arm to actually push for max extension
                armSystem.Wait(0.75),
                armSystem.RunMethod("openClaw"),
                armSystem.Wait(0.5),
                armSystem.RunMethod("setWrist", 0, 90.0),
                armSystem.Wait(0.5)
        );

        SequentialAction DropPickupSample1 = new SequentialAction(
                armSystem.RunMethod("setWristToFloorPickup"),
                armSystem.RunMethod("closeClaw", 0.4),
                armSystem.RunMethod("setWristToRaisedFloor", 0.7),
                armSystem.Wait(1)
        );
        SequentialAction DropPickupSample2 = new SequentialAction(
                armSystem.RunMethod("setWristToFloorPickup"),
                armSystem.RunMethod("closeClaw", 0.4),
                armSystem.RunMethod("setWristToRaisedFloor", 0.7),
                armSystem.Wait(1)
        );
        SequentialAction DropPickupSample3 = new SequentialAction(
                armSystem.RunMethod("setWristToFloorPickup"),
                armSystem.RunMethod("closeClaw", 0.4),
                armSystem.RunMethod("setWristToRaisedFloor", 0.7),
                armSystem.Wait(1)
        );



        // NOTE make sure any methods that need parameters are correct and all numbers MUST BE doubles with a decimal place

        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToBack, setWristToStraight, setWristToFloorPickup, setWristToRaisedFloor, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw, dropSamplePickup
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, holdClawToFieldCoordinate, moveArmDirectly, setWrist, setExtension, setPivot

        // Actions:
        // Wait, RunMethod ^, waitUntilFinishedAwaiting, waitUntilFinishedMoving

        drive.getRRDrive().updatePoseEstimate();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        armSystem.RunMethod("closeClaw"),
                        armSystem.RunMethod("setWristToStraight"),
                        armSystem.Wait(0.2),
                        armSystem.RunMethod("moveArmDirectly", 0.0, 76.0, 205.0),
                        armSystem.Wait(0.2),
                        DriveToChamber1.build(),
                        armSystem.RunMethod("moveClawToTopRung"),
                        PlaceSpecimen,
                        armSystem.RunMethod("resetArm", 0.3),

                        armSystem.RunMethod("setWristToRaisedFloor", 0.6),
                        armSystem.RunMethod("moveArmDirectly", 1.0, 5.0, 250.0),
                        DriveToSample1.build(),
                        armSystem.Wait(1),
                        DropPickupSample1,
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("moveClawToTopBasket", 0.2),
                        DriveToBasket1.build(),
                        DumpSampleInBasket1,
                        armSystem.RunMethod("resetArm", 0.75),

                        armSystem.RunMethod("setWristToRaisedFloor", 1.0),
                        armSystem.RunMethod("moveArmDirectly", 1.25, 5.0, 250.0),
                        DriveToSample2.build(),
                        armSystem.Wait(1),
                        DropPickupSample2,
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("moveClawToTopBasket", 0.2),
                        DriveToBasket2.build(),
                        DumpSampleInBasket2,
                        armSystem.RunMethod("resetArm", 0.75),

                        armSystem.RunMethod("setWristToRaisedFloor", 1.0),
                        armSystem.RunMethod("moveArmDirectly", 1.25, 5.0, 250.0),
                        DriveToSample3.build(),
                        armSystem.Wait(1),
                        DropPickupSample3,
                        armSystem.Wait(0.25),
                        armSystem.RunMethod("moveClawToTopBasket", 0.2),
                        DriveToBasket3.build(),
                        DumpSampleInBasket3,
                        armSystem.RunMethod("resetArm", 0.75),

                        ParkAtRungs.build(),
                        armSystem.Wait(1)

                ));

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
