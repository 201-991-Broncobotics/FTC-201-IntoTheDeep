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
import org.firstinspires.ftc.teamcode.Roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.List;

public class DriveAutonCommand extends CommandBase {

    DifferentialSwerveDrive drive;

    private final FtcDashboard dash = FtcDashboard.getInstance();
    public static List<Action> runningActions = new ArrayList<>();

    Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));

    ElapsedTime AutonUpdateSpeedTimer;

    Telemetry telemetry;

    public DriveAutonCommand(DifferentialSwerveDrive drivetrain, Telemetry telemetry) {
        addRequirements(drivetrain);
        drive = drivetrain;
        this.telemetry = telemetry;
        AutonUpdateSpeedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        SubsystemData.repeatForwardBack = false; // make sure this is off if not specifically needed
    }


    @Override
    public void execute() {
        telemetry.addLine("Auton frameRate:" + (1 / (AutonUpdateSpeedTimer.time() / 1000)));
        AutonUpdateSpeedTimer.reset();
        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet) && !SubsystemData.inTeleOp) { // && !SubsystemData.driver.getButton(GamepadKeys.Button.A)
                newActions.add(action);
            }
        }
        runningActions = newActions;



        if (runningActions.isEmpty() && !SubsystemData.inTeleOp && SubsystemData.repeatForwardBack) {
            runningActions.add(new SequentialAction(
                    drive.actionBuilder(startPose)
                            .strafeTo(new Vector2d(0, 30))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(0, 0))
                            .waitSeconds(1)
                            //.turnTo(Math.toRadians(270))
                            //.waitSeconds(1)
                            //.turnTo(Math.toRadians(90))
                            .build()
            ));
        }





        telemetry.addLine("Error X:" + functions.round(SubsystemData.AutonError.position.x, 3) + " Y:" + functions.round(SubsystemData.AutonError.position.y, 3) + " A:" + functions.round(Math.toDegrees(SubsystemData.AutonError.heading.toDouble()), 3));

        // drive.updateDifferentialSwerve(); // keep drivetrain running

        // telemetry.update();

        /*
        if (SubsystemData.driver.getButton(GamepadKeys.Button.A)) { // locks the entire code so I can read the telemetry
            while (Thread.currentThread().isAlive()) {
                drive.stopDifferentialSwerve();
            }
        }

         */

        SubsystemData.LastAutonPose = drive.pose; // keeps track of the pose during auton and saves it for teleOp

        dash.sendTelemetryPacket(packet);
    }


    public static void queueAction(Action action) {
        runningActions.add(action);
    }


    @Override
    public boolean isFinished() {
        return SubsystemData.inTeleOp;
    }

}
