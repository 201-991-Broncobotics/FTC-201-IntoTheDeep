package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Time;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SubsystemData;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class SwerveModule {

    private final PIDController modulePID;

    private final DoubleSupplier topMotorEncoder;

    private final DcMotorEx topMotor, bottomMotor;

    private double ModuleZeroAngle;

    private double targetAngle;


    public SwerveModule(DcMotorEx newTopMotor, DcMotorEx newBottomMotor, double startingAngle) { // initialize the module
        ModuleZeroAngle = startingAngle;
        targetAngle = startingAngle;
        topMotor = newTopMotor;
        bottomMotor = newBottomMotor;
        topMotorEncoder = () -> functions.angleDifference((topMotor.getCurrentPosition() / Constants.encoderResolution * 360) - ModuleZeroAngle, 0, 360);
        modulePID = new PIDController(0, 0, 0, topMotorEncoder);
        modulePID.setSettingsTheSameAs(SubsystemData.SwerveModuleReferencePID);
    }


    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }


    public void setCurrentDiffyAngleTo(double newAngleZero) { ModuleZeroAngle = newAngleZero; }


    public void setModule(double angle, double speed, double maxPowerLimit) {
        targetAngle = angle;
        modulePID.setSettingsTheSameAs(SubsystemData.SwerveModuleReferencePID);
        if (SubsystemData.SwerveModuleDriveSharpness < 1) SubsystemData.SwerveModuleDriveSharpness = 1;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        if (Math.abs(speed) > 1) speed = Math.signum(speed); // module shouldn't try to calculate speeds well beyond 1 as it messes stuff up

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        double DriveSharpnessCurve = Math.sin(((Math.abs(functions.angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2);
        speed = speed * (Math.signum(DriveSharpnessCurve) * Math.abs(Math.pow(DriveSharpnessCurve, SubsystemData.SwerveModuleDriveSharpness)));

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -1 * speed + rotation;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        topMotor.setPower(-1 * R1Power / divider);
        bottomMotor.setPower(-1 * R2Power / divider);
    }

    public void setModuleDual(double angle, DualNum<Time> speed, double maxPowerLimit, MotorFeedforward feedForward, double voltage) {
        targetAngle = angle;
        modulePID.setSettingsTheSameAs(SubsystemData.SwerveModuleReferencePID);
        if (SubsystemData.SwerveModuleDriveSharpness < 1) SubsystemData.SwerveModuleDriveSharpness = 1;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        double DriveSharpnessCurve = Math.sin(((Math.abs(functions.angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2);
        speed = speed.times(Math.signum(DriveSharpnessCurve) * Math.abs(Math.pow(DriveSharpnessCurve, SubsystemData.SwerveModuleDriveSharpness)));

        // maintain the correct motor speed balance
        DualNum<Time> R1Power = speed.plus(rotation);
        DualNum<Time> R2Power = speed.times(-1).plus(rotation);
        double divider = Math.max(1, Math.max(R1Power.value() / maxPowerLimit, R2Power.value() / maxPowerLimit));

        topMotor.setPower(feedForward.compute(R1Power.times(-1).div(divider)) / voltage);
        bottomMotor.setPower(feedForward.compute(R2Power.times(-1).div(divider)) / voltage);
        //topMotor.setPower(R1Power.value() * -1 / divider);
        //bottomMotor.setPower(R2Power.value() * -1 / divider);
    }

    public void fullStopModule() {
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    /*
    public boolean isCloseEnough() {
        return (Math.abs(functions.angleDifference(topMotorEncoder.getAsDouble(), targetAngle, 180)) < SubsystemData.SwerveModuleTolerance);
    }

     */

}
