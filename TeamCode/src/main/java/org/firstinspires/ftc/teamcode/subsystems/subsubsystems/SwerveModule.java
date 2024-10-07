package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveModule {

    private final MotorEx top_motor, bottom_motor;

    private final PIDController modulePID;


    public SwerveModule(HardwareMap map, String top_motor_name, String bottom_motor_name) { // initialize the module
        top_motor = new MotorEx(map, top_motor_name);
        bottom_motor = new MotorEx(map, bottom_motor_name);
        top_motor.setRunMode(Motor.RunMode.RawPower);
        bottom_motor.setRunMode(Motor.RunMode.RawPower);

        modulePID = new PIDController(0.01, 0, 0, () -> functions.angleDifference(top_motor.getCurrentPosition() / Constants.encoderResolution * 360, 0, 360));
    }


    public double getCurrentAngle() {
        return functions.angleDifference(top_motor.getCurrentPosition() / Constants.encoderResolution * 360, 0, 360);
    }

    public void setModule(double angle, double speed, double throttle) {
        // find current angle in degrees of the wheel and wrap it to between -90 and 90
        double currentAngle = getCurrentAngle();
        double angleChange = functions.angleDifference(currentAngle, angle, 180);

        // if angle change is big enough, create a difference in power between the motors based on how big the change is
        double rotation = 0;
        //if (speed > 0 && Math.abs(angleDifference) > 2) { // error range in degrees
        //angleChange = 1.5 * (angleDifference / 360) + 0.1; // balance, constant amount
        //}
        rotation = modulePID.getPower(0.0, angleChange);

        // rate at which the wheel attempts to realign itself
        speed = speed * Math.sin(((Math.abs(functions.angleDifference(currentAngle, angle, 360)) / 90) - 1) * Math.PI / 2);

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -speed + rotation;
        double divider = Math.max(1, Math.max(R1Power, R2Power));

        top_motor.set(-1 * R1Power / divider * throttle);
        bottom_motor.set(-1 * R2Power / divider * throttle);
    }


}
