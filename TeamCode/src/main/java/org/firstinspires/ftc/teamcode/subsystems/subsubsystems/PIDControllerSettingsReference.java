package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

public class PIDControllerSettingsReference {

    public double kP, kI, kD, minPosition, maxPosition, minPower, maxPower, initialPower, minDifference, maxSpeed, tolerance, maxAcceleration, maxDeceleration;

    public boolean positionLimitingEnabled, speedLimitingEnabled;

    /**
     * All positions and speeds are in the same units as the doubleSupplier of the encoder values and speed/acceleration is based on seconds
     *
     * @param aP P value (the "a" makes it appear at the top in ftcdashboard)
     * @param aI I value
     * @param aD D value
     * @param minPosition lower position limit (if positionLimitingEnabled)
     * @param maxPosition upper position limit (if positionLimitingEnabled)
     * @param minPower
     * @param maxPower
     * @param initialPower power needed to start turning the motor / make sure to set minPower if using
     * @param minDifference stops PID when the error is less than this
     * @param maxSpeed (if speedLimitingEnabled)
     * @param tolerance +-this is the range where closeEnough() will return true
     * @param maxAcceleration (if speedLimitingEnabled)
     * @param maxDeceleration (if speedLimitingEnabled)
     * @param positionLimitingEnabled
     * @param speedLimitingEnabled
     */
    public PIDControllerSettingsReference(double aP, double aI, double aD, double minPosition, double maxPosition, double minPower, double maxPower, double initialPower, double minDifference, double maxSpeed, double tolerance, double maxAcceleration, double maxDeceleration, boolean positionLimitingEnabled, boolean speedLimitingEnabled) {
        kP = aP;
        kI = aI;
        kD = aD;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.initialPower = initialPower;
        this.minDifference = minDifference;
        this.maxSpeed = maxSpeed;
        this.tolerance = tolerance;
        // this.maxIntegral = maxIntegral;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.positionLimitingEnabled = positionLimitingEnabled;
        this.speedLimitingEnabled = speedLimitingEnabled;
    }

}
