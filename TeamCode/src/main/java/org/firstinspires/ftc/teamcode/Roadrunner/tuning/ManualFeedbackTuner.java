package org.firstinspires.ftc.teamcode.Roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.Roadrunner.TwoDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(DifferentialSwerveDrive.class)) {
            DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)), telemetry);
            
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            PoseVelocity2d currentPose = drive.updatePoseEstimate();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(currentPose.linearVel.x, currentPose.linearVel.y, currentPose.angVel))
                            .strafeToConstantHeading(new Vector2d(currentPose.linearVel.x, currentPose.linearVel.y + DISTANCE))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(currentPose.linearVel.x, currentPose.linearVel.y))
                            .waitSeconds(0.5)
                            .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0), telemetry);

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToY(DISTANCE)
                            .lineToY(0)
                            .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
