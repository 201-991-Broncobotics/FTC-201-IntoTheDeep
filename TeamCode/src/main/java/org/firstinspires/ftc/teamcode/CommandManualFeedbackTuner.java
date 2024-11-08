package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;


@TeleOp(name="CommandManualFeedbackTuner")
public class CommandManualFeedbackTuner extends CommandOpMode {

    public static double DISTANCE = 20;

    @Override
    public void initialize() {

        // assume a position of 0, 0, 0 is in the exact center of the field pointing away from audience

        Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // always running
        drive.setDefaultCommand(new DriveAutonCommand(drive));

        schedule(new RunCommand(telemetry::update)); // update telemetry needs to be scheduled last as the commands are executed in the order they were scheduled

        waitForStart();

        PoseVelocity2d currentPose = drive.updatePoseEstimate();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                            .strafeToConstantHeading(new Vector2d(0, DISTANCE))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(0, 0))
                            .waitSeconds(0.5)
                            .build());
        }

    }

}
