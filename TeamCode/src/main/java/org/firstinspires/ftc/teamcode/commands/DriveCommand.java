package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.DifferentialSwerveDrivetrain;

public class DriveCommand extends RunCommand {
    public DriveCommand(DifferentialSwerveDrivetrain drivetrain) {
        super(drivetrain::controlDifferentialSwerve, drivetrain);
    }
}