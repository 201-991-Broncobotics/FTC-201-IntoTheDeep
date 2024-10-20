package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class SwerveModule {

    //private final DcMotorEx top_motor, bottom_motor;

    private final PIDController modulePID;

    private DualNum<Time> topMotorPower, bottomMotorPower;

    private final DoubleSupplier topMotorEncoder;


    private double LastRotation;


    public SwerveModule(DcMotorEx top_motor /*HardwareMap map, String top_motor_name, String bottom_motor_name*/) { // initialize the module
        /*
        top_motor = map.get(DcMotorEx.class, top_motor_name);
        bottom_motor = map.get(DcMotorEx.class, bottom_motor_name);
        top_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset only the rotation encoder
        top_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottom_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        */
        topMotorEncoder = () -> functions.angleDifference(top_motor.getCurrentPosition() / Constants.encoderResolution * 360, 0, 360);
        modulePID = new PIDController(0.01, 0, 0, topMotorEncoder);
    }


    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }

    public void setModule(double angle, DualNum<Time> speed) {
        // find current angle in degrees of the wheel and wrap it to between -90 and 90
        double currentAngle = getCurrentAngle();
        double angleChange = functions.angleDifference(currentAngle, angle, 180);

        double rotation = modulePID.getPower(0.0, angleChange);
        LastRotation = rotation;

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        speed = speed.times(Math.sin(((Math.abs(functions.angleDifference(currentAngle, angle, 360)) / 90) - 1) * Math.PI / 2));

        // maintain the correct motor speed balance
        DualNum<Time> R1Power = speed.plus(rotation);
        DualNum<Time> R2Power = speed.times(-1).plus(rotation);
        double divider = Math.max(1, Math.max(R1Power.value(), R2Power.value()));

        topMotorPower = R1Power.times(-1).div(divider);
        bottomMotorPower = R2Power.times(-1).div(divider);
    }


    public DualNum<Time> getTopMotorPower() { return topMotorPower; }
    public DualNum<Time> getBottomMotorPower() { return bottomMotorPower; }

    public double getRotation() { return LastRotation; }

}
