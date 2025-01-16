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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;
import java.util.List;

public class DriveAutonCommand extends CommandBase {

    Follower drive; // now the implementation for pedro pathing driving



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
        telemetry.addLine("Auton frameRate:" + (1 / (AutonUpdateSpeedTimer.time() / 1000)));
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

        // Gives all the times that I logged inside parts of the code so I can optimise
        if (SubsystemData.loggedTimes.isEmpty()) telemetry.addLine("No times logged");
        else {
            for (int i = 0; i < SubsystemData.loggedTimes.size(); i++) {
                telemetry.addData("Log " + (i + 1) + ":" + SubsystemData.loggedTimes.get(i) + " - ", SubsystemData.loggedMessages.get(i));
            }
        }
        SubsystemData.logReset();



        // telemetry.addLine("Error X:" + functions.round(SubsystemData.AutonError.position.x, 3) + " Y:" + functions.round(SubsystemData.AutonError.position.y, 3) + " A:" + functions.round(Math.toDegrees(SubsystemData.AutonError.heading.toDouble()), 3));
        // telemetry.update();

        // drive.updateDifferentialSwerve(); // keep drivetrain running

        // telemetry.update();

        /*
        if (SubsystemData.driver.getButton(GamepadKeys.Button.A)) { // locks the entire code so I can read the telemetry
            while (Thread.currentThread().isAlive()) {
                drive.stopDifferentialSwerve();
            }
        }

         */

        // SubsystemData.CurrentRobotPose = drive.pose; // keeps track of the pose during auton and saves it for teleOp

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
