package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.DiffySwerve;

public class DriveCommand extends RunCommand {
    public DriveCommand(DiffySwerve drivetrain) {
        super(drivetrain::controlDifferentialSwerve, drivetrain);
    }
}