package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.commands.BreakInSwerveModulesCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExplosionSystem;


@TeleOp(name="BreakInSwerveModulesControl", group="Utility")
public class BreakInSwerveModulesControl extends CommandOpMode {

    @Override
    public void initialize() {

        SubsystemData.inTeleOp = true;

        SubsystemData.driver = new GamepadEx(gamepad1);
        SubsystemData.operator = new GamepadEx(gamepad2);

        ExplosionSystem kaboom = new ExplosionSystem(hardwareMap, telemetry);

        kaboom.setDefaultCommand(new BreakInSwerveModulesCommand(kaboom));

        waitForStart();

    }
}
