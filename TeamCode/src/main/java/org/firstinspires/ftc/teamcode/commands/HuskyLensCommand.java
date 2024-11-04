package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.HuskyLensCamera;


public class HuskyLensCommand extends RunCommand {
    public HuskyLensCommand(HuskyLensCamera Camera) {
        super(Camera::ScanForSample, Camera);
    }
}

/*
public class HuskyLensCommand extends CommandBase {

    private final HuskyLensCamera HuskyLensSubsystem;

    public HuskyLensCommand(HuskyLensCamera subsystem) {
        HuskyLensSubsystem = subsystem;
        addRequirements(HuskyLensSubsystem);
    }

    @Override
    public void initialize() {
        HuskyLensSubsystem.StartHuskyLensThread();
    }

    @Override
    public void execute() {
        HuskyLensSubsystem.ScanForSample();
    }

    @Override
    public boolean isFinished() {
        HuskyLensSubsystem.EndHuskyLensThread();
        return true;
    }

}

 */