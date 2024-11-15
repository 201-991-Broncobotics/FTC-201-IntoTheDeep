package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.commands.ArmClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp(name="ResetSwerveWheels", group="Utility")
public class ResetSwerveWheelsOpMode extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = true;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);

        drive.resetSwerveWheelAngles();

        waitForStart();

        telemetry.addLine("Swerve Wheels have been reset to 0 :)");
        telemetry.update();

    }
}
