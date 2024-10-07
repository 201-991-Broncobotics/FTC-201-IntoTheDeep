package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import java.util.function.DoubleSupplier;

public class PIDController {

    private final double kP, kI, kD, minPosition, maxPosition, minPower, maxPower, maxSpeed, tolerance;

    private boolean positionLimitingEnabled = false, speedLimitingEnabled = false;

    private final DoubleSupplier encoderPosition;

    private double integral, previousTime, targetPosition, movingTargetPosition, lastError;


    // full PID with all of the possible settings
    public PIDController(double kP, double kI, double kD, double minPosition, double maxPosition,
                         double minPower, double maxPower, double maxSpeed, double tolerance,
                         boolean positionLimitingEnabled, boolean speedLimitingEnabled, DoubleSupplier encoderPosition) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.minPosition = minPosition; // doesn't matter what this is if position limiting is false
        this.maxPosition = maxPosition; // doesn't matter what this is if position limiting is false
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.maxSpeed = maxSpeed; // doesn't matter what this is if speed limiting is false
        this.tolerance = tolerance; // the range where closeEnough() will return true
        this.positionLimitingEnabled = positionLimitingEnabled;
        this.speedLimitingEnabled = speedLimitingEnabled;
        this.encoderPosition = encoderPosition;
        reset();
    }


    // simplified versions of calling this PID controller
    public PIDController(double kP, double kI, double kD, DoubleSupplier encoderPosition) { // simplest version
        this(kP, kI, kD, 0, 0, 0, 1, 0, 0, false, false, encoderPosition);
    }
    public PIDController(double kP, double kI, double kD, double minPosition, double maxPosition, DoubleSupplier encoderPosition) { // just position limiting
        this(kP, kI, kD, minPosition, maxPosition, 0, 1, 0, 0, true, false, encoderPosition);
    }
    public PIDController(double kP, double kI, double kD, double maxSpeed, DoubleSupplier encoderPosition) { // just speed limiting
        this(kP, kI, kD, 0, 0, 0, 1, maxSpeed, 0, false, true, encoderPosition);
    }
    public PIDController(double kP, double kI, double kD, double minPosition, double maxPosition, double maxSpeed, DoubleSupplier encoderPosition) { // position and speed limiting
        this(kP, kI, kD, minPosition, maxPosition, 0, 1, maxSpeed, 0, true, true, encoderPosition);
    }
    // there probably is a way to make each value have a default value and only overwrite if you define it without copying this line 11^11 different times but idk what it is


    public void setTarget(double newTargetPosition) {
        if (positionLimitingEnabled) { // make sure target is always between max and min limits if enabled
            if (newTargetPosition > maxPosition) newTargetPosition = maxPosition;
            else if (newTargetPosition < minPosition) newTargetPosition = minPosition;
        }
        targetPosition = newTargetPosition;
        previousTime = System.currentTimeMillis() / 1000.0;
        integral = 0; // reset integral so it doesn't go crazy
    }


    public void reset() {
        targetPosition = encoderPosition.getAsDouble();
        movingTargetPosition = targetPosition;
        previousTime = System.currentTimeMillis() / 1000.0;
        integral = 0;
    }


    // this limits the speed by making the target position, that the PID is aiming for, approach the set target position at the max speed
    private void updateMovingTargetPosition() {
        if (speedLimitingEnabled) {
            if (movingTargetPosition < targetPosition) { // move the movingTargetPosition at maxSpeed towards the set targetPosition until the targetPosition is reached
                movingTargetPosition += maxSpeed;
                if (movingTargetPosition > targetPosition) movingTargetPosition = targetPosition;
            } else if (movingTargetPosition > targetPosition) {
                movingTargetPosition -= maxSpeed;
                if (movingTargetPosition < targetPosition) movingTargetPosition = targetPosition;
            }
        } else movingTargetPosition = targetPosition; // if not speed limiting movingTargetPosition is just targetPosition
    }


    public double getPower(double overrideCurrentPosition) { // needs to be called constantly to work properly
        double power;
        double timeSince = (System.currentTimeMillis() / 1000.0) - previousTime;

        updateMovingTargetPosition();

        double Error = movingTargetPosition - overrideCurrentPosition; // calculate PID values
        integral += Error * timeSince;
        double Derivative = (Error - lastError) / timeSince;
        lastError = Error;
        previousTime = System.currentTimeMillis() / 1000.0;
        power =  (Error * kP) + (integral * kI) + (Derivative * kD); // calculate the result


        // make sure power obeys all of the set limits
        if (Math.abs(power) > maxPower) power = Math.signum(power) * maxPower;
        else if (Math.abs(power) < minPower) power = 0.0;

        // prevent motor from continuing to move outside of min and max limits if enabled
        if (positionLimitingEnabled) {
            if (overrideCurrentPosition < minPosition && power < 0) power = 0;
            else if (overrideCurrentPosition > maxPosition && power > 0) power = 0;
        }

        return power;
    }


    public double getPower() { // normal call which just sets current position to current position
        return getPower(encoderPosition.getAsDouble()); // this allows using the PID by setting the targetPosition to 0 and the amount it needs to change by to the overrideCurrentPosition
    }


    public double getPower(double targetPosition, double currentPosition) { // combine setTarget and getPower
        setTarget(targetPosition);
        return getPower(currentPosition);
    }


    public boolean closeEnough() {
        return Math.abs(targetPosition - encoderPosition.getAsDouble()) <= tolerance;
    }

}
