package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final DcMotorEx R1, R2, R3, R4, Pivot, Extension;


    public final IMU imu;


    public double minimumMotorSpeed = 0.4;


    public double encoderResolution = 8192; // resolution for 1 module turn

    public double lastRightAngle = 0;

    public double lastLeftAngle = 0;


    // PID variables
    double RightPosIntegralSum = 0, PosKp = 0.01, PosKi = 0, PosKd = 0; // Right Diffy
    private double RightPosLastError = 0;
    ElapsedTime RightPIDtimer = new ElapsedTime();

    // left diffy
    double LeftPosIntegralSum = 0; // Kp Ki and Kd are the same for each diffy module
    private double LeftPosLastError = 0;
    ElapsedTime LeftPIDtimer = new ElapsedTime();

    // Pivot
    double PivotIntegralSum = 0, PivotKp = 0.01, PivotKi = 0, PivotKd = 0;
    private double PivotLastError = 0;
    ElapsedTime PivotPIDtimer = new ElapsedTime();

    // Extension
    double ExtendIntegralSum = 0, ExtendKp = 0.01, ExtendKi = 0, ExtendKd = 0;
    private double ExtendLastError = 0;
    ElapsedTime ExtendPIDtimer = new ElapsedTime();




    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));


        R1 = hardwareMap.get(DcMotorEx.class, "R1"); // R1,R2 Module Encoder
        R2 = hardwareMap.get(DcMotorEx.class, "R2");
        R3 = hardwareMap.get(DcMotorEx.class, "R3");
        R4 = hardwareMap.get(DcMotorEx.class, "R4");
        Pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        Extension = hardwareMap.get(DcMotorEx.class, "Extension");


        R1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        R3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        R4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        R1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        R2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        R3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        R4.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Pivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Extension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addLine("Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;

    } // initializes everything


    // finds the smallest difference between two angles or wraps an angle to between -180 and 180 when target is 0 (when wrapAngle is 360)
    // wrapAngle of 180 treats the targetAngle and the angle opposite of targetAngle the same
    public double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }


    public void methodSleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds while in a method
        }
    }


    public void driveDiffySwerveWithControllers(double direction, double magnitude, double turn, double throttle) {
        double forward = Math.sin(Math.toRadians(direction)) * magnitude; // convert vector to x and y
        double strafe = Math.cos(Math.toRadians(direction)) * magnitude;

        double A = -forward - turn;
        double B = -forward + turn;
        double RightPower = Math.sqrt(strafe * strafe + A * A);
        double LeftPower = Math.sqrt(strafe * strafe + B * B);
        double max_power = Math.max(1, Math.max(RightPower, LeftPower)); // keeps all motor powers under 1
        RightPower = RightPower / max_power; // target motor speeds
        LeftPower = LeftPower / max_power;
        double RightAngle = Math.toDegrees(Math.atan2(strafe, A)); // Target wheel angles
        double LeftAngle = Math.toDegrees(Math.atan2(strafe, B));

        throttle = 1 - ((1 - minimumMotorSpeed) * throttle);

        // actually tell the pod to go to the angle at the power
        if (Math.abs(strafe) > 0.05 || Math.abs(forward) > 0.05 || Math.abs(turn) > 0.05) {
            rightDiffPod(RightAngle, RightPower, throttle);
            leftDiffPod(LeftAngle, LeftPower, throttle);
            lastRightAngle = RightAngle;
            lastLeftAngle = LeftAngle;
        } else { // when no controller input, stop moving wheels
            rightDiffPod(lastRightAngle, 0, throttle);
            leftDiffPod(lastLeftAngle, 0, throttle);
        }

    }


    public double getCurrentRightDiffAngle() {
        return angleDifference(R1.getCurrentPosition() / encoderResolution * 360, 0, 360);
    }


    public double getCurrentLeftDiffAngle() {
        return angleDifference(R3.getCurrentPosition() / encoderResolution * 360, 0, 360);
    }


    public void rightDiffPod(double angle, double speed, double throttle) {
        // find current angle in degrees of the wheel and wrap it to between -90 and 90
        double currentAngle = getCurrentRightDiffAngle();
        double angleChange = angleDifference(currentAngle, angle, 180);

        // if angle change is big enough, create a difference in power between the motors based on how big the change is
        double rotation = 0;
        //if (speed > 0 && Math.abs(angleDifference) > 2) { // error range in degrees
            //angleChange = 1.5 * (angleDifference / 360) + 0.1; // balance, constant amount
        //}

        rotation = rightPosPID(0, angleChange);

        // rate at which the wheel attempts to realign itself
        speed = speed * Math.sin(((Math.abs(angleDifference(currentAngle, angle, 360)) / 90) - 1) * Math.PI / 2);

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -speed + rotation;
        double divider = Math.max(1, Math.max(R1Power, R2Power));

        R1.setPower(-1 * R1Power / divider * throttle);
        R2.setPower(-1 * R2Power / divider * throttle);

    }


    public void leftDiffPod(double angle, double speed, double throttle) {
        // find current angle in degrees of the wheel and wrap it to between -90 and 90
        double currentAngle = getCurrentLeftDiffAngle();
        double angleChange = angleDifference(currentAngle, angle, 180);

        // if angle change is big enough, create a difference in power between the motors based on how big the change is
        double rotation = 0;
        rotation = leftPosPID(0, angleChange);

        speed = speed * Math.sin(((Math.abs(angleDifference(currentAngle, angle, 360)) / 90) - 1) * Math.PI / 2);
        double R3Power = speed + rotation;
        double R4Power = -speed + rotation;
        double divider = Math.max(1, Math.max(R3Power, R4Power));

        R3.setPower(-1 * R3Power / divider * throttle);
        R4.setPower(-1 * R4Power / divider * throttle);

    }


    public double rightPosPID(double RightPosReference, double RightPosState) { // Reference is the target, state is current position (i copied this from internet)
        double RightPosError = RightPosReference - RightPosState;
        RightPosIntegralSum += RightPosError * RightPIDtimer.seconds();
        double RightPosDerivative = (RightPosError - RightPosLastError) / RightPIDtimer.seconds();
        RightPosLastError = RightPosError;

        RightPIDtimer.reset();

        return (RightPosError * PosKp) + (RightPosDerivative * PosKd) + (RightPosIntegralSum * PosKi);
    }

    public double leftPosPID(double LeftPosReference, double LeftPosState) {
        double LeftPosError = LeftPosReference - LeftPosState;
        LeftPosIntegralSum += LeftPosError * LeftPIDtimer.seconds();
        double LeftPosDerivative = (LeftPosError - LeftPosLastError) / LeftPIDtimer.seconds();
        LeftPosLastError = LeftPosError;

        LeftPIDtimer.reset();

        return (LeftPosError * PosKp) + (LeftPosDerivative * PosKd) + (LeftPosIntegralSum * PosKi);
    }

    public double PivotPID(double Target, double Current) {
        double Error = Target - Current;
        PivotIntegralSum += Error * PivotPIDtimer.seconds();
        double Derivative = (Error - PivotLastError) / PivotPIDtimer.seconds();
        PivotLastError = Error;

        PivotPIDtimer.reset();

        return (Error * PivotKp) + (Derivative * PivotKd) + (PivotIntegralSum * PivotKi);
    }

    public double ExtensionPID(double Target, double Current) {
        double Error = Target - Current;
        ExtendIntegralSum += Error * ExtendPIDtimer.seconds();
        double Derivative = (Error - ExtendLastError) / ExtendPIDtimer.seconds();
        ExtendLastError = Error;

        ExtendPIDtimer.reset();

        return (Error * ExtendKp) + (Derivative * ExtendKd) + (ExtendIntegralSum * ExtendKi);
    }

}
