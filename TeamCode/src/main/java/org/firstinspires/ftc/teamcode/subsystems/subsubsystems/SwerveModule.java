package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemData;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class SwerveModule {

    private final PIDController modulePID;

    private final DoubleSupplier topMotorEncoder;

    private final DcMotorEx topMotor, bottomMotor;


    public SwerveModule(DcMotorEx newTopMotor, DcMotorEx newBottomMotor) { // initialize the module
        topMotor = newTopMotor;
        bottomMotor = newBottomMotor;
        topMotorEncoder = () -> functions.angleDifference(topMotor.getCurrentPosition() / Constants.encoderResolution * 360, 0, 360);
        SubsystemData.SwerveModuleKp = 0.005;
        SubsystemData.SwerveModuleKi = 0;
        SubsystemData.SwerveModuleKd = 0;
        modulePID = new PIDController(0.005, 0, 0, topMotorEncoder);
    }


    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }


    public void setModule(double angle, double speed, double maxPowerLimit, double voltage) {
        modulePID.kP = SubsystemData.SwerveModuleKp;
        modulePID.kI = SubsystemData.SwerveModuleKi;
        modulePID.kD = SubsystemData.SwerveModuleKd;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        speed = speed * (Math.sin(((Math.abs(functions.angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2));

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -1 * speed + rotation;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        topMotor.setPower(-1 * R1Power / divider / voltage);
        bottomMotor.setPower(-1 * R2Power / divider / voltage);
    }

}
