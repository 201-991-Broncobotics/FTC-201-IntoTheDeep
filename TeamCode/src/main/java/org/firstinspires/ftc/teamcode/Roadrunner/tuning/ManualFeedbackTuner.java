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

    public static double AxialGain = 2.0;
    public static double LateralGain = 2.0;
    public static double HeadingGain = 25.0;
    public static double AxialVelocityGain = 0;
    public static double LateralVelocityGain = 0;
    public static double HeadingVelocityGain = 0;

    public static double MaxWheelVel = 30;
    public static double MinProfileAccel = -30;
    public static double MaxProfileAccel = 30;
    public static double pauseSeconds = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(DifferentialSwerveDrive.class)) {
            Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(0));
            DifferentialSwerveDrive drive = new DifferentialSwerveDrive(hardwareMap, startingPose, telemetry);
            
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

            drive.updatePoseEstimate();

            while (opModeIsActive()) {
                drive.updateFeedBackParameters(AxialGain, LateralGain, HeadingGain, AxialVelocityGain, LateralVelocityGain, HeadingVelocityGain);
                drive.updatePathParameters(MaxWheelVel, MinProfileAccel, MaxProfileAccel);
                Actions.runBlocking(
                    drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(startingPose.position.x + DISTANCE, startingPose.position.y))
                            .waitSeconds(pauseSeconds)
                            .strafeToConstantHeading(startingPose.position)
                            .waitSeconds(pauseSeconds)
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
