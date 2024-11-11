package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.List;

public class DriveAutonCommand extends CommandBase {

    DifferentialSwerveDrive drive;

    private final FtcDashboard dash = FtcDashboard.getInstance();
    public static List<Action> runningActions = new ArrayList<>();

    Telemetry telemetry;

    public DriveAutonCommand(DifferentialSwerveDrive drivetrain, Telemetry telemetry) {
        addRequirements(drivetrain);
        drive = drivetrain;
        this.telemetry = telemetry;
    }


    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet) && !SubsystemData.driver.getButton(GamepadKeys.Button.A)) { // also makes the auton cancellable
                newActions.add(action);
            }
        }
        runningActions = newActions;

        telemetry.addLine("Error X:" + functions.round(SubsystemData.AutonError.position.x, 3) + " Y:" + functions.round(SubsystemData.AutonError.position.y, 3) + " A:" + functions.round(Math.toDegrees(SubsystemData.AutonError.heading.toDouble()), 3));

        drive.updateDifferentialSwerve(); // keep drivetrain running

        dash.sendTelemetryPacket(packet);
    }

}
