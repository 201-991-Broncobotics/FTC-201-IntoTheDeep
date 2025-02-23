package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExplosionSystem;

public class BreakInSwerveModulesCommand extends RunCommand {
    public BreakInSwerveModulesCommand(ExplosionSystem explosionSystem) {
        super(explosionSystem::runControls, explosionSystem);
    }
}