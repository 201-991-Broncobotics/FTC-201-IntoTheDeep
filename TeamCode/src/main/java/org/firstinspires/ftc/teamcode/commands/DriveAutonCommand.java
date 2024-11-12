package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));

    ElapsedTime AutonUpdateSpeed;

    Telemetry telemetry;

    public DriveAutonCommand(DifferentialSwerveDrive drivetrain, Telemetry telemetry) {
        addRequirements(drivetrain);
        drive = drivetrain;
        this.telemetry = telemetry;
        AutonUpdateSpeed = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }


    @Override
    public void execute() {
        telemetry.addData("Auton frameRate:", 1 / AutonUpdateSpeed.time() / 1000);
        AutonUpdateSpeed.reset();
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

        if (runningActions.isEmpty()) {
            runningActions.add(new SequentialAction(
                    drive.actionBuilder(startPose)
                            .strafeToConstantHeading(new Vector2d(0, 10))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(0, 0))
                            .waitSeconds(0.5)
                            //.turnTo(Math.toRadians(270))
                            //.waitSeconds(1)
                            .build()
            ));
        }

        telemetry.addLine("Error X:" + functions.round(SubsystemData.AutonError.position.x, 3) + " Y:" + functions.round(SubsystemData.AutonError.position.y, 3) + " A:" + functions.round(Math.toDegrees(SubsystemData.AutonError.heading.toDouble()), 3));

        drive.updateDifferentialSwerve(); // keep drivetrain running

        telemetry.update();

        if (SubsystemData.driver.getButton(GamepadKeys.Button.A)) {
            while (Thread.currentThread().isAlive()) {
                drive.stopDifferentialSwerve();
            }
        }

        dash.sendTelemetryPacket(packet);
    }

}
