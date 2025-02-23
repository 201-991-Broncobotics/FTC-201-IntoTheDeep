package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.SubsystemData;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class SwerveModule {

    private final PIDController modulePID;

    private final DoubleSupplier topMotorEncoder;

    private final DcMotorEx topMotor, bottomMotor;

    private PhotonAdvancedDcMotor topMotorAdv, bottomMotorAdv;

    private double ModuleZeroAngle;

    private boolean usePhoton = false;
    private double LastR1Power = 0, LastR2Power = 0;


    public SwerveModule(DcMotorEx newTopMotor, DcMotorEx newBottomMotor, double startingAngle) { // initialize the module
        ModuleZeroAngle = startingAngle;
        topMotor = newTopMotor;
        bottomMotor = newBottomMotor;
        topMotorEncoder = () -> functions.angleDifference((topMotor.getCurrentPosition() / Constants.encoderResolution * 360) - ModuleZeroAngle, 0, 360);
        modulePID = new PIDController(0, 0, 0, topMotorEncoder);
        modulePID.setSettingsTheSameAs(Settings.SwerveReference);
    }


    public void zeroSwerveModule() {
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ModuleZeroAngle = (topMotor.getCurrentPosition() / Constants.encoderResolution * 360);
        modulePID.replaceDoubleSupplier(topMotorEncoder);
    }


    public void turnToZero(double maxPowerLimit) {
        modulePID.setSettingsTheSameAs(Settings.SwerveReference);
        modulePID.kI = Settings.SwerveAlignmentKI;

        double rotation = modulePID.getPowerWrapped(0, 360);

        // maintain the correct motor speed balance
        double divider = Math.max(1, Math.abs(rotation / maxPowerLimit));

        if (usePhoton) {
            topMotorAdv.setPower(-1 * rotation / divider);
            bottomMotorAdv.setPower(-1 * rotation / divider);
        } else {
            topMotor.setPower(-1 * rotation / divider);
            bottomMotor.setPower(-1 * rotation / divider);
        }
    }



    public void usePhotonAdvancedDcMotors(PhotonAdvancedDcMotor newTopMotorAdv, PhotonAdvancedDcMotor newBottomMotorAdv) {
        topMotorAdv = newTopMotorAdv;
        bottomMotorAdv = newBottomMotorAdv;
        usePhoton = true;
    }


    public double getCurrentAngle() {
        return topMotorEncoder.getAsDouble();
    }


    public void setCurrentDiffyAngleTo(double newAngleZero) { ModuleZeroAngle = newAngleZero; }


    public void setModule(double angle, double speed, double maxPowerLimit) {
        modulePID.setSettingsTheSameAs(Settings.SwerveReference);
        if (Settings.SwerveModuleDriveSharpness < 1) Settings.SwerveModuleDriveSharpness = 1;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        if (Math.abs(speed) > 1) speed = Math.signum(speed); // module shouldn't try to calculate speeds beyond 1 as it could mess the math up

        if (Math.abs(speed) > 0) speed = (1 - Settings.driveFeedBackStaticPower) * speed + Math.signum(speed) * Settings.driveFeedBackStaticPower;

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        double DriveSharpnessCurve = Math.sin(((Math.abs(functions.angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2);
        speed = speed * (Math.signum(DriveSharpnessCurve) * Math.abs(Math.pow(DriveSharpnessCurve, Settings.SwerveModuleDriveSharpness)));

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -1 * speed + rotation;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        R1Power = -1 * R1Power / divider;
        R2Power = -1 * R2Power / divider;

        if (Math.abs(LastR1Power - R1Power) >= Settings.DriveMotorReadDifference || (R1Power == 0 && !(LastR1Power == 0))) {
            topMotor.setPower(R1Power);
            LastR1Power = R1Power;
        }
        if (Math.abs(LastR2Power - R2Power) >= Settings.DriveMotorReadDifference || (R2Power == 0 && !(LastR2Power == 0))) {
            bottomMotor.setPower(R2Power);
            LastR2Power = R2Power;
        }
    }

    public void tuneDriveStaticPower(double maxPowerLimit) {
        modulePID.setSettingsTheSameAs(Settings.SwerveReference);
        if (Settings.SwerveModuleDriveSharpness < 1) Settings.SwerveModuleDriveSharpness = 1;
        double angle = 0;
        double speed = Settings.driveFeedBackStaticPower;

        double rotation = modulePID.getPowerWrapped(angle, 180);

        if (Math.abs(speed) > 1) speed = Math.signum(speed); // module shouldn't try to calculate speeds beyond 1 as it could mess the math up

        // rate at which the wheel attempts to realign itself vs power diverted towards moving forward
        double DriveSharpnessCurve = Math.sin(((Math.abs(functions.angleDifference(getCurrentAngle(), angle, 360)) / 90) - 1) * Math.PI / 2);
        speed = speed * (Math.signum(DriveSharpnessCurve) * Math.abs(Math.pow(DriveSharpnessCurve, Settings.SwerveModuleDriveSharpness)));

        // maintain the correct motor speed balance
        double R1Power = speed + rotation;
        double R2Power = -1 * speed + rotation;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        R1Power = -1 * R1Power / divider;
        R2Power = -1 * R2Power / divider;

        topMotor.setPower(R1Power);
        bottomMotor.setPower(R2Power);

    }


    public void fullStopModule() {
        if (usePhoton) {
            topMotorAdv.setPower(0);
            bottomMotorAdv.setPower(0);
        } else {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }
    }


    public void setMotorZeroBehavior(DcMotor.ZeroPowerBehavior mode) {
        if (!(mode == topMotor.getZeroPowerBehavior())) {
            topMotor.setZeroPowerBehavior(mode);
            bottomMotor.setZeroPowerBehavior(mode);
        }
    }

    /*
    public boolean isCloseEnough() {
        return (Math.abs(functions.angleDifference(topMotorEncoder.getAsDouble(), targetAngle, 180)) < SubsystemData.SwerveModuleTolerance);
    }

     */

}


/*
Simplified version for readability:



    // finds the smallest difference between two angles or gets the equivalent angle between -180 and
    // 180 when target is set to 0 (and wrapAngle is 360)
    // A wrapAngle of 180 would return the smallest distance to the targetAngle or the angle
    // directly opposite of targetAngle, depending on which one is closer
    public double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }

    public void setModule(double angle, double speed, double maxPowerLimit) {
        // prevent module from trying to calculate speeds beyond 1 as it could mess the ratios up
        if (Math.abs(speed) > 1) speed = Math.signum(speed);

        // finds closest distance to target angle or directly opposite target angle and gets rotation PID power
        double changeInAngle = angleDifference(getCurrentAngle(), angle, 180); // in degrees from actual target
        double rotationPower = modulePID.getPower(0, changeInAngle);

        // Slow down drive speed only when wheel is facing in the wrong direction so power can be
        // diverted to rotating the module. Also reverses power if module is targeting the opposite angle
        speed = speed * Math.cos(Math.toRadians(
        angleDifference(getCurrentAngle(), angle, 360)
        ));

        // combine the speed and rotation but keep it no greater than 1 for correct ratio
        double R1Power = rotationPower + speed;
        double R2Power = rotationPower - speed;
        double divider = Math.max(1, Math.max(R1Power / maxPowerLimit, R2Power / maxPowerLimit));

        topMotor.setPower(R1Power / divider);
        bottomMotor.setPower(R2Power / divider);
    }

 */