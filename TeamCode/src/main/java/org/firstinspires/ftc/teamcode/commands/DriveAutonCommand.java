package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DiffySwerve;

public class DriveAutonCommand extends RunCommand {
    public DriveAutonCommand(MecanumDrive drivetrain) {
        super(drivetrain::updateDifferentialSwerve, drivetrain);
    }
}
