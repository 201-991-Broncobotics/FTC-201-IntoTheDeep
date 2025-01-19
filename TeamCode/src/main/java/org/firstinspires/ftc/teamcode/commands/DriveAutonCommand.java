package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.List;

public class DriveAutonCommand extends CommandBase {

    Follower drive; // now the implementation for pedro pathing driving


    private double AutonTime = 0;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    public static List<Action> runningActions = new ArrayList<>();

    Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));

    ElapsedTime AutonUpdateSpeedTimer;

    Telemetry telemetry;

    public DriveAutonCommand(Follower drivetrain, Telemetry telemetry) {
        addRequirements(drivetrain);
        drive = drivetrain;
        this.telemetry = telemetry;
        AutonUpdateSpeedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        SubsystemData.repeatForwardBack = false; // make sure this is off if not specifically needed
        SubsystemData.inTeleOp = false; // TODO: move this once I add auto driving in TeleOp if needed
    }


    @Override
    public void execute() {
        telemetry.addLine("Pedro Target " + functions.TilePoseAsString(functions.PedroToRRPose(SubsystemData.TargetPedroPose)));
        telemetry.addLine("Auton Time:" + AutonTime);
        AutonUpdateSpeedTimer.reset();
        TelemetryPacket packet = new TelemetryPacket();

        SubsystemData.inTeleOp = false; // TODO: move this once I add auto driving in TeleOp if needed

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet) && !SubsystemData.inTeleOp) { // && !SubsystemData.driver.getButton(GamepadKeys.Button.A)
                newActions.add(action);
            }
        }
        runningActions = newActions;

        drive.update();

        /*
        if (SubsystemData.driver.getButton(GamepadKeys.Button.A)) { // locks the entire code so I can read the telemetry
            while (Thread.currentThread().isAlive()) {
                drive.stopDifferentialSwerve();
            }
        }
         */

        // drive.telemetryDebugWithoutUpdate(telemetry);
        dash.sendTelemetryPacket(packet);
        AutonTime = AutonUpdateSpeedTimer.time();
    }


    public static void queueAction(Action action) {
        runningActions.add(action);
    }

    public static Runnable queueActionRunnable(Action action) {
        runningActions.add(action);
        return null;
    }


    @Override
    public boolean isFinished() {
        return SubsystemData.inTeleOp;
    }

}
