package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DifferentialSwerveDrivetrain extends SubsystemBase {

    private static SwerveModule rightModule, leftModule;

    // private final DoubleSupplier ForwardSupplier, StrafeSupplier, TurnSupplier, ThrottleSupplier;

    private static double lastRightAngle = 0.0;
    private static double lastLeftAngle = 0.0;

    private static DualNum<Time> RightTop, RightBottom, LeftTop, LeftBottom;

    public boolean absoluteDriving = true;

    public DifferentialSwerveDrivetrain(DcMotorEx topRightMotor, DcMotorEx topLeftMotor/*HardwareMap map, GamepadEx gamepad*/) {
        rightModule = new SwerveModule(topRightMotor/*map, "R2", "R1"*/); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(topLeftMotor/*map, "R3", "R4"*/);
        /* ForwardSupplier = gamepad::getLeftY;
        StrafeSupplier = gamepad::getLeftX;
        TurnSupplier = gamepad::getRightX;
        ThrottleSupplier = () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

         */
    }

    public static void setupDiffy(DcMotorEx topRightMotor, DcMotorEx topLeftMotor) {
        rightModule = new SwerveModule(topRightMotor/*map, "R2", "R1"*/); // rotation encoders need to be the top motors for consistency and in ports 2 and 3 since port 0 and 3 on the control hub are more accurate for odometry
        leftModule = new SwerveModule(topLeftMotor/*map, "R3", "R4"*/);
    }

    public void controlDifferentialSwerve() {
        /*
        double forward = ForwardSupplier.getAsDouble(); // get initial values
        double strafe = StrafeSupplier.getAsDouble();
        double turn = TurnSupplier.getAsDouble();
        double throttle = ThrottleSupplier.getAsDouble();
        double heading = 0; // TODO: needs to be set to the correct twist value from roadrunner

        // normalize values to make it easier for the driver to control
        double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
        double joystickMagnitude = Math.hypot(strafe, forward);
        double drivePower = Math.abs(joystickMagnitude) * joystickMagnitude;
        double throttleControl = 0.5 + throttle / 2;

        if (absoluteDriving) {
            driveDirection = functions.angleDifference(driveDirection + heading, 0, 360);
        }

        // ask roadrunner to run mecanum drive instead
        // driveDifferentialSwerve(driveDirection, drivePower, -0.4 * Math.abs(turn) * turn, throttleControl);

         */
    }

    /*
    public void controlDifferentialSwerve() {
        double forward = ForwardSupplier.getAsDouble(); // get initial values
        double strafe = StrafeSupplier.getAsDouble();
        double turn = TurnSupplier.getAsDouble();
        double throttle = ThrottleSupplier.getAsDouble();
        double heading = 0;

        // normalize values to make it easier for the driver to control
        double driveDirection = Math.toDegrees(Math.atan2(forward, strafe));
        double joystickMagnitude = Math.hypot(strafe, forward);
        double drivePower = Math.abs(joystickMagnitude) * joystickMagnitude;
        double throttleControl = 0.5 + throttle / 2;

        if (absoluteDriving) {
            driveDirection = functions.angleDifference(driveDirection + heading, 0, 360);
        }

        // ask roadrunner to run mecanum drive instead
        // driveDifferentialSwerve(driveDirection, drivePower, -0.4 * Math.abs(turn) * turn, throttleControl);
    }
    */

    public static void driveDifferentialSwerve(PoseVelocity2dDual<Time> command, double trackWidth) {
        DualNum<Time> forward = command.linearVel.y;
        DualNum<Time> strafe = command.linearVel.x;
        DualNum<Time> turn = command.angVel.times(trackWidth);

        DualNum<Time> A = forward.times(-1).minus(turn); // diffy swerve drive math
        DualNum<Time> B = forward.times(-1).plus(turn);
        DualNum<Time> RightPower = ((strafe.times(strafe)).plus((A.times(A)))).sqrt(); // who knows if this will work
        DualNum<Time> LeftPower = ((strafe.times(strafe)).plus((B.times(B)))).sqrt();

        double max_power = Math.max(1, Math.max(RightPower.value(), LeftPower.value())); // keeps all motor powers under 1
        RightPower = RightPower.div(max_power); // target motor speeds
        LeftPower = LeftPower.div(max_power);
        double RightAngle = Math.toDegrees(Math.atan2(strafe.value(), A.value())); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe.value(), B.value()));


        // actually tell the pod to go to the angle at the power
        if (Math.abs(strafe.value()) > 0 || Math.abs(forward.value()) > 0 || Math.abs(turn.value()) > 0) {
            rightModule.setModule(RightAngle, RightPower);
            leftModule.setModule(LeftAngle, LeftPower);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightModule.setModule(lastRightAngle, RightPower);
            leftModule.setModule(lastLeftAngle, LeftPower);
        }

        RightTop = rightModule.getTopMotorPower();
        RightBottom = rightModule.getBottomMotorPower();
        LeftTop = leftModule.getTopMotorPower();
        LeftBottom = leftModule.getBottomMotorPower();
    }


    public void toggleAbsoluteDriving() {
        absoluteDriving = !absoluteDriving;
    }


    // idk how to create an array of "Dual<Time>"s to return with
    public static DualNum<Time> getRightTop() { return RightTop; }
    public static DualNum<Time> getRightBottom() { return RightBottom; }
    public static DualNum<Time> getLeftTop() { return LeftTop; }
    public static DualNum<Time> getLeftBottom() { return LeftBottom; }


}
