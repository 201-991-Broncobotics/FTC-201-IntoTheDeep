package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.Localization;

public class UpdateLocalizerCommand extends RunCommand {
    public UpdateLocalizerCommand(Localization localizerSystem) {
        super(localizerSystem::updateLocalizer, localizerSystem);
    }
}