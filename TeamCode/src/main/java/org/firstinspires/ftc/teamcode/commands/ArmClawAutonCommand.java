package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;

public class ArmClawAutonCommand extends RunCommand {
    public ArmClawAutonCommand(ArmSystem armClaw) {
        super(armClaw::updateClawArm, armClaw);
    }
}
