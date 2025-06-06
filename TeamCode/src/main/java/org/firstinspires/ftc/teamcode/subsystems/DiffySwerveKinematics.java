package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.ArrayList;

public class DiffySwerveKinematics extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    private static double lastRightAngle = 0.0, lastLeftAngle = 0.0, maxPower = 1;

    private static PoseVelocity2d driveCommand; // for roadrunner
    private static double TrackWidth, Voltage; // voltage allows auton to drive at the same power even at low battery
    private static MotorFeedforward FeedForward;

    private static Telemetry telemetry;

    private static final ArrayList<PoseVelocity2dDual<Time>> LastCommands = new ArrayList<PoseVelocity2dDual<Time>>();

    private static boolean AlignSwerveModulesToZero = false;


    public DiffySwerveKinematics(DcMotorEx newLeftTop, DcMotorEx newLeftBottom, DcMotorEx newRightBottom, DcMotorEx newRightTop, double maxPowerLimit, Telemetry newTelemetry) {
        maxPower = maxPowerLimit; // helps to slow down how fast the gears wear down
        telemetry = newTelemetry;

        rightModule = new SwerveModule(newRightTop, newRightBottom, Constants.driveWheelBaseAngle, 1); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(newLeftTop, newLeftBottom, Constants.driveWheelBaseAngle, 1);

        telemetry.addData("Current right Diffy Angle:", rightModule.getCurrentAngle());
        telemetry.addData("Current left Diffy Angle:", leftModule.getCurrentAngle());
        telemetry.update();

        driveCommand = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }


    public void switchToPhotonAdvancedDcMotors(PhotonAdvancedDcMotor newLeftTopPhoton, PhotonAdvancedDcMotor newLeftBottomPhoton, PhotonAdvancedDcMotor newRightBottomPhoton, PhotonAdvancedDcMotor newRightTopPhoton) {
        rightModule.usePhotonAdvancedDcMotors(newRightTopPhoton, newRightBottomPhoton);
        leftModule.usePhotonAdvancedDcMotors(newLeftTopPhoton, newLeftBottomPhoton);
    }


    // only use one of these diffy swerve methods at one time as some of the values are shared
    public void driveDifferentialSwerve(double forward, double strafe, double turn, double throttle) {
        // Compensate for if the diffy wheels aren't symmetrical to each side
        double driveDirection = Math.atan2(forward, strafe) + Math.toRadians(Constants.driveWheelBaseAngle);
        double drivePower = Math.hypot(forward, strafe);
        forward = Math.sin(driveDirection) * drivePower;
        strafe = Math.cos(driveDirection) * drivePower;

        double A = -forward + turn; // diffy swerve drive math
        double B = -forward - turn;
        double RightPower = Math.hypot(strafe, A);
        double LeftPower = Math.hypot(strafe, B);

        maxPower = Settings.maxDrivetrainMotorPower;

        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = (RightPower / max_power) * throttle; // target motor speeds
        LeftPower = (LeftPower / max_power) * throttle;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));

        // actually tell the pod to go to the angle at the power
        if (Settings.tuneDriveFeedBackStaticPower) {
            rightModule.tuneDriveStaticPower(maxPower);
            leftModule.tuneDriveStaticPower(maxPower);
            lastRightAngle = 0;
            lastLeftAngle = 0;
            AlignSwerveModulesToZero = false;
        } else if (Math.abs(strafe) > 0 || Math.abs(forward) > 0 || Math.abs(turn) > 0) {
            rightModule.setModule(RightAngle, RightPower, maxPower);
            leftModule.setModule(LeftAngle, LeftPower, maxPower);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
            AlignSwerveModulesToZero = false;
        } else if (AlignSwerveModulesToZero) {
            rightModule.turnToZero(maxPower);
            leftModule.turnToZero(maxPower);
            lastRightAngle = 0;
            lastLeftAngle = 0;
            telemetry.addData("Current right Diffy Angle:", rightModule.getCurrentAngle());
            telemetry.addData("Current left Diffy Angle:", leftModule.getCurrentAngle());
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, RightPower, maxPower);
            leftModule.setModule(lastLeftAngle, LeftPower, maxPower);
        }

        //telemetry.addLine("test Right Module Powers: T" + rightModule.getTopMotorPower() + " B: " + rightModule.getBottomMotorPower());
        //telemetry.addLine("test Left Module Powers: T" + leftModule.getTopMotorPower() + " B: " + leftModule.getBottomMotorPower());
    }


    public void setDifferentialSwerve(double forward, double strafe, double turn) {
        driveCommand = new PoseVelocity2d(new Vector2d(strafe, forward), turn);
        // updateKinematicDifferentialSwerve();
    }

    public void setDifferentialSwerve(PoseVelocity2d command) {
        driveCommand = command;
        // updateKinematicDifferentialSwerve();
    }

    public void updateKinematicDifferentialSwerve() {
        driveDifferentialSwerve(driveCommand.linearVel.y, driveCommand.linearVel.x, functions.capValue(driveCommand.angVel, Settings.maxDrivetrainTurnPower), 1);
    }


    public void stopDifferentialSwerve() {
        driveCommand = new PoseVelocity2d(new Vector2d(0, 0), 0);
        // DriveCommand.HeadingTargetPID.stopUntilNextUse();
        rightModule.fullStopModule();
        leftModule.fullStopModule();
    }


    public void setDriveMotorsZeroBehavior(DcMotor.ZeroPowerBehavior mode) {
        rightModule.setMotorZeroBehavior(mode);
        leftModule.setMotorZeroBehavior(mode);
    }

    public void zeroSwerveModules() {
        rightModule.zeroSwerveModule();
        leftModule.zeroSwerveModule();
        lastRightAngle = 0;
        lastLeftAngle = 0;
    }

    public void alignSwerveModulesToZero() {
        AlignSwerveModulesToZero = true;
    }


    /*

    double testingAngle = 0;
    double testingTime = 5000; // milliseconds per module of testing
    double testingPower = 0.25;

    boolean testing2ndSide = false;

    double newRightAngle = 0, newLeftAngle = 0;

    double fastestAngularVelocity = 0;

    Pose2d TestingStartPose;

    ElapsedTime TestingTimer;

    ElapsedTime TestingFrameRateTimer;

    public void emergencyAlignDiffySwerve() {
        stopDifferentialSwerve();
        TestingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        TestingFrameRateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        fastestAngularVelocity = 0;
        TestingStartPose = SubsystemData.CurrentRobotPose;
        DiffyEmergencyRealigning = true;
    }

    public void updateEmergencyAlignDiffySwerve() {
        double UpdateSpeed = TestingFrameRateTimer.time(); // how many milliseconds between each update
        TestingFrameRateTimer.reset();

        if (testingAngle > 180 && !testing2ndSide) { // switch to other side
            testingAngle = 0;
            fastestAngularVelocity = 0;
            testing2ndSide = true;
        } else if (testingAngle > 180) { // disable emergency aligning and save new values
            testing2ndSide = false;
            testingAngle = 0;
            fastestAngularVelocity = 0;
            DiffyEmergencyRealigning = false;
        }


        if (testingAngle < 180) {
            if (testing2ndSide) { // test left side if on 2nd side
                leftModule.setModule(testingAngle, testingPower, maxPower);
                rightModule.fullStopModule();

                newLeftAngle = testingAngle;
            } else {
                rightModule.setModule(testingAngle, testingPower, maxPower);
                leftModule.fullStopModule();



                newRightAngle = testingAngle;


            }

        } else {
            TestingTimer.reset(); // switch to next angle to test after it has been enough time
        }

        testingAngle += (180 / testingTime) * UpdateSpeed;

    }

     */

}
