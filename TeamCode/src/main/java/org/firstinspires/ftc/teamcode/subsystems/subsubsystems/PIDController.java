package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class PIDController {

    public double kP, kI, kD, minPosition, maxPosition, minPower, maxPower, maxSpeed, tolerance; // all of these variables can be change elsewhere in the code
    public double maxIntegral = 1; // helps prevent integral from constantly adding up if the mechanism is stuck

    public boolean positionLimitingEnabled = false, speedLimitingEnabled = false, speedLimitingOverride = false;

    public final DoubleSupplier encoderPosition; // also allows getting the mechanism's current position with .encoderPosition.getAsDouble()

    private double integral, previousTime, targetPosition, movingTargetPosition, lastError, activeMinPosition, activeMaxPosition;
    private double percentMaxSpeed = 1;

    ElapsedTime mRuntime;


    // full PID with all of the possible settings
    public PIDController(double kP, double kI, double kD, double minPosition, double maxPosition,
                         double minPower, double maxPower, double maxSpeed, double tolerance,
                         boolean positionLimitingEnabled, boolean speedLimitingEnabled, DoubleSupplier encoderPosition) { // units are the same as the units of the encoderPosition
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.minPosition = minPosition; // doesn't matter what this is if position limiting is false
        this.maxPosition = maxPosition; // doesn't matter what this is if position limiting is false
        activeMaxPosition = maxPosition;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.maxSpeed = maxSpeed; // doesn't matter what this is if speed limiting is false
        this.tolerance = tolerance; // the range where closeEnough() will return true
        this.positionLimitingEnabled = positionLimitingEnabled;
        this.speedLimitingEnabled = speedLimitingEnabled;
        this.encoderPosition = encoderPosition;
        mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        reset();
    }


    // simplified versions of calling this PID controller
    public PIDController(double kP, double kI, double kD, DoubleSupplier encoderPosition) { // simplest version
        this(kP, kI, kD, 0, 0, 0, 1, 0, 0, false, false, encoderPosition);
    }
    // there probably is a way to make each value have a default value and only overwrite if you define it without copying this line 11^11 different times but idk what it is


    public void setTarget(double newTargetPosition) {
        speedLimitingOverride = false; // if target is set without specifying speed limiter override, reset to off
        if (positionLimitingEnabled) { // makes sure target is always between max and min limits if enabled
            if (newTargetPosition > activeMaxPosition) newTargetPosition = activeMaxPosition;
            else if (newTargetPosition < activeMinPosition) newTargetPosition = activeMinPosition;
        }
        targetPosition = newTargetPosition;
        previousTime = mRuntime.time() / 1000.0;
        integral = 0; // reset integral so it doesn't go crazy
    }


    public void setTarget(double newTargetPosition, boolean OverrideSpeedLimiter) { // override only works if speedLimitingEnabled is set to true
        speedLimitingOverride = OverrideSpeedLimiter;
        if (positionLimitingEnabled) { // makes sure target is always between max and min limits if enabled
            if (newTargetPosition > activeMaxPosition) newTargetPosition = activeMaxPosition;
            else if (newTargetPosition < activeMinPosition) newTargetPosition = activeMinPosition;
        }
        targetPosition = newTargetPosition;
        previousTime = mRuntime.time() / 1000.0;
        integral = 0; // reset integral so it doesn't go crazy
    }


    public void reset() {
        targetPosition = encoderPosition.getAsDouble();
        movingTargetPosition = targetPosition;
        previousTime = mRuntime.time() / 1000.0;
        integral = 0;
    }


    // this limits the speed by making the target position, that the PID is aiming for, approach the set target position at the max speed
    private void updateMovingTargetPosition() {
        if (speedLimitingEnabled && !speedLimitingOverride) {
            if (movingTargetPosition < targetPosition) { // move the movingTargetPosition at maxSpeed towards the set targetPosition until the targetPosition is reached
                movingTargetPosition += maxSpeed * percentMaxSpeed;
                if (movingTargetPosition > targetPosition) movingTargetPosition = targetPosition;
            } else if (movingTargetPosition > targetPosition) {
                movingTargetPosition -= maxSpeed * percentMaxSpeed;
                if (movingTargetPosition < targetPosition) movingTargetPosition = targetPosition;
            }
        } else movingTargetPosition = targetPosition; // if not speed limiting movingTargetPosition is just targetPosition
    }


    public double getPower(double overrideCurrentPosition) { // VERY IMPORTANT that this needs to be called constantly to work properly
        // overrideCurrentPosition is just the current encoder Position unless it is being overridden
        correctValues();
        double power;
        double timeSince = (mRuntime.time() / 1000.0) - previousTime;

        updateMovingTargetPosition();

        double Error = movingTargetPosition - overrideCurrentPosition; // calculate PID values
        integral += Error * timeSince;
        if (Math.abs(integral) > maxIntegral) integral = Math.signum(integral) * maxIntegral; // stabilize integral
        double Derivative = (Error - lastError) / timeSince;
        lastError = Error;
        previousTime = mRuntime.time() / 1000.0;
        power =  (Error * kP) + (integral * kI) + (Derivative * kD); // calculate the result


        // make sure power obeys all of the set limits
        if (Math.abs(power) > maxPower) power = Math.signum(power) * maxPower;
        else if (Math.abs(power) < minPower) power = 0.0;

        // prevent motor from continuing to move outside of min and max limits if enabled
        if (positionLimitingEnabled) {
            if (overrideCurrentPosition < activeMinPosition && power < 0) power = 0;
            else if (overrideCurrentPosition > activeMaxPosition && power > 0) power = 0;
        }

        return power;
    }


    private void correctValues() { // kind of unnecessary but makes there be less steps when creating code to edit these on the fly
        if (kP < 0) kP = 0;
        if (kI < 0) kI = 0;
        if (kD < 0) kD = 0;
        if (minPosition > maxPosition) { // flip max and min values when the min is above the max
            activeMinPosition = maxPosition;
            activeMaxPosition = minPosition;
        } else {
            activeMinPosition = minPosition;
            activeMaxPosition = maxPosition;
        }
        if (minPower < 0) minPower = 0;
        if (maxPower > 1) maxPower = 1;
        if (minPower > maxPower) minPower = maxPower;
        if (maxSpeed < 0) maxSpeed = 0;
        if (tolerance < 0) tolerance = Math.abs(tolerance);
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


    public void setPercentMaxSpeed(double PercentMaxSpeed) { // between 0 to 1 and only works when speed limiting is on
        percentMaxSpeed = PercentMaxSpeed;
    }

}
