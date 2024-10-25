package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;

public class HuskyLensCommand extends RunCommand {
    public HuskyLensCommand(HuskyLensCamera Camera) {
        super(Camera::ScanForSample, Camera);
    }
}