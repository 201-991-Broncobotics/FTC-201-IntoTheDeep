package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;


@TeleOp(name="ResetWrist", group="Utility")
public class ResetWristServoOpMode extends CommandOpMode {

    Servo Wrist;

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = true;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        //DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);

        //drive.resetSwerveWheelAngles();

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        if (Settings.ArmSystemSettings.WristServoReversed) Wrist.setDirection(Servo.Direction.REVERSE);
        else Wrist.setDirection(Servo.Direction.FORWARD);
        Wrist.setPosition(1 - (Settings.ArmSystemSettings.WristServoRatio * ((0 + Settings.ArmSystemSettings.WristServoOffset) / 360.0)));

        telemetry.addLine("Wrist have been reset successfully");
        telemetry.update();

        waitForStart();

        requestOpModeStop();

    }
}
