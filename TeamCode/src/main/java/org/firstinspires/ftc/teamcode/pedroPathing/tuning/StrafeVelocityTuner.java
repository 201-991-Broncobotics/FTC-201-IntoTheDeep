package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.tileCoords;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the StrafeVelocityTuner autonomous tuning OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 * You can adjust the distance the robot will travel on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous(name = "Strafe Velocity Tuner", group = "Autonomous Pathing Tuning")
public class StrafeVelocityTuner extends OpMode {
    private ArrayList<Double> velocities = new ArrayList<>();

    private DifferentialSwerveDrive swerveDrive;

    /*
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

     */

    private PoseUpdater poseUpdater;

    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private Telemetry telemetryA;

    private boolean end;

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
     */
    @Override
    public void init() {
        Pose2d startPose = new Pose2d(tileCoords(0, 0), Math.toRadians(90));
        swerveDrive = new DifferentialSwerveDrive(hardwareMap, startPose, telemetry);
        poseUpdater = new PoseUpdater(hardwareMap, swerveDrive);

        swerveDrive.setDifferentialSwerveZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        // TODO: Make sure that this is the direction your motors need to be reversed in.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

         */

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.");
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetryA.update();
    }

    /**
     * This starts the OpMode by setting the drive motors to run right at full power.
     */
    @Override
    public void start() {
        //leftFront.setPower(1);
        //leftRear.setPower(-1);
        //rightFront.setPower(-1);
        //rightRear.setPower(1);
        swerveDrive.setDifferentialSwerve(0, 1 ,0);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            /*
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }

             */
            swerveDrive.setDifferentialSwerveZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            swerveDrive.stopDifferentialSwerve();
            requestOpModeStop();
        }
        swerveDrive.updateDifferentialSwerve();

        poseUpdater.update();
        if (!end) {
            if (Math.abs(poseUpdater.getPose().getY()) > DISTANCE) {
                end = true;
                /*
                for (DcMotorEx motor : motors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor.setPower(0);
                }
                 */
                swerveDrive.setDifferentialSwerveZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                swerveDrive.stopDifferentialSwerve();
            } else {
                double currentVelocity = Math.abs(MathFunctions.dotProduct(poseUpdater.getVelocity(), new Vector(1, Math.PI / 2)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            //leftFront.setPower(0);
            //leftRear.setPower(0);
            //rightFront.setPower(0);
            //rightRear.setPower(0);

            //for (DcMotorEx motor : motors) {
            //    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //}
            swerveDrive.setDifferentialSwerveZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            swerveDrive.stopDifferentialSwerve();
            double average = 0;
            for (Double velocity : velocities) {
                average += velocity;
            }
            average /= (double) velocities.size();

            telemetryA.addData("strafe velocity:", average);
            telemetryA.update();
        }
    }
}
