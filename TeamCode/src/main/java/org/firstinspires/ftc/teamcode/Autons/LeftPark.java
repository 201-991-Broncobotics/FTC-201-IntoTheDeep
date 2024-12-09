package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.commands.ArmClawAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.commands.HuskyLensCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PedroTrajectoryActionBuilder;


@Autonomous(name="LeftPark")
public class LeftPark extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = false;

        // assume a position of 0, 0, 0 would be at the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(tiles(-0.5), tiles(-3) + 7.09), Math.toRadians(90));
        Follower drive = new Follower(hardwareMap, startPose, telemetry);
        drive.startTeleopDrive();
        ArmSystem armSystem = new ArmSystem(hardwareMap, telemetry);
        HuskyLensCamera HuskyLensSystem = new HuskyLensCamera(hardwareMap);

        // always running
        // drive.setDefaultCommand(new DriveAutonCommand(drive, telemetry));
        schedule(new DriveAutonCommand(drive, telemetry));
        HuskyLensSystem.setDefaultCommand(new HuskyLensCommand(HuskyLensSystem));
        armSystem.setDefaultCommand(new ArmClawAutonCommand(armSystem));


        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled


        // setup roadrunner trajectories
        PedroTrajectoryActionBuilder DriveToLeft = drive.actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(-2.7, -2.4));



        // ArmClaw method names:
        // openClaw, closeClaw, toggleClaw, setWristToCenter, setWristToBasket, setWristToFloorPickup, depositSpecimen
        // enableLoosenClaw, disableLoosenClaw, toggleLoosenClaw,
        // moveClawToTopBasket, moveClawToTopRung, moveClawToRamRung, moveClawToHumanPickup, resetArm,

        // Parameter methods:
        // moveArmToPoint, moveClawToFieldCoordinate, moveArmDirectly, setWrist

        // Actions:
        // Wait, RunMethod ^, waitUntilFinishedAwaiting, waitUntilFinishedMoving

        drive.getRRDrive().updatePoseEstimate();

        DriveAutonCommand.queueAction(
                new SequentialAction(
                        DriveToLeft.build(),
                        armSystem.Wait(1)
                )
        );

        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

}
