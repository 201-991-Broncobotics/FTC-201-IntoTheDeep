package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class PIDController {

    public double kP, kI, kD, minPosition, maxPosition, minPower, maxPower, initialPower, maxSpeed, tolerance; // all of these variables can be changed elsewhere in the code

    public boolean positionLimitingEnabled = false, speedLimitingEnabled = false, speedLimitingOverride = false;

    public final DoubleSupplier encoderPosition; // also allows getting the mechanism's current position with .encoderPosition.getAsDouble()

    private double integral, previousTime, targetPosition, movingTargetPosition, lastError, activeMinPosition, activeMaxPosition;
    private double maxIntegral = 1, percentMaxSpeed = 1, PIDFrameRate = 0;

    private boolean stoppedUntilNextUse = false;

    ElapsedTime mRuntime;


    // full PID with all of the possible settings
    public PIDController(double kP, double kI, double kD, double minPosition, double maxPosition,
                         double minPower, double maxPower, double initialPower, double maxSpeed, double tolerance,
                         boolean positionLimitingEnabled, boolean speedLimitingEnabled, DoubleSupplier encoderPosition) { // units are the same as the units of the encoderPosition
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.minPosition = minPosition; // doesn't matter what this is if position limiting is false, same units as the doubleSupplier
        this.maxPosition = maxPosition; // doesn't matter what this is if position limiting is false, same units as the doubleSupplier
        this.minPower = minPower; // can be used as the initial power needed to start moving
        this.maxPower = maxPower;
        this.initialPower = initialPower; // power needed to start turning the motor / SET A MINIMUM POWER WHEN THIS IS GREATER THAN 0 or the PID will oscillate
        this.maxSpeed = maxSpeed; // IF SET TO 0, MAX SPEED WILL BE IGNORED, also doesn't matter what this is if speed limiting is false, in units per second
        this.tolerance = tolerance; // the range where closeEnough() will return true
        this.positionLimitingEnabled = positionLimitingEnabled;
        this.speedLimitingEnabled = speedLimitingEnabled;
        this.encoderPosition = encoderPosition;
        mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // default minPower value when it isn't set and initial power is enabled
        if (minPower == 0 && initialPower > 0) this.minPower = initialPower * 2;

        correctValues();
        if (kI > 0) maxIntegral = maxPower / kI;

        reset();
    }


    // simple version of this PID controller
    public PIDController(double kP, double kI, double kD, DoubleSupplier encoderPosition) { // simplest version
        this(kP, kI, kD, 0, 0, 0, 1, 0, 0, 0, false, false, encoderPosition);
    }
    // there probably is a way to make each value have a default value and only overwrite if you define it without copying this line 11^11 different times but idk what it is


    public void setTarget(double newTargetPosition) {
        speedLimitingOverride = false; // if target is set without specifying speed limiter override, reset to off
        if (positionLimitingEnabled) { // makes sure target is always between max and min limits if enabled
            if (newTargetPosition > maxPosition) newTargetPosition = maxPosition;
            else if (newTargetPosition < minPosition) newTargetPosition = minPosition;
        }
        if (!(targetPosition == newTargetPosition)) integral = 0; // reset integral so it doesn't go crazy
        targetPosition = newTargetPosition;
    }


    public void setTarget(double newTargetPosition, boolean OverrideSpeedLimiter) { // override only works if speedLimitingEnabled is set to true
        speedLimitingOverride = OverrideSpeedLimiter;
        if (positionLimitingEnabled) { // makes sure target is always between max and min limits if enabled
            if (newTargetPosition > maxPosition) newTargetPosition = maxPosition;
            else if (newTargetPosition < minPosition) newTargetPosition = minPosition;
        }
        if (!(targetPosition == newTargetPosition)) integral = 0; // reset integral so it doesn't go crazy
        targetPosition = newTargetPosition;
    }


    public void reset() {
        targetPosition = encoderPosition.getAsDouble();
        movingTargetPosition = targetPosition;
        previousTime = mRuntime.time() / 1000.0;
        integral = 0;
    }


    // this limits the speed by making the target position, that the PID is aiming for, approach the set target position at the max speed
    private void updateMovingTargetPosition() {
        if (speedLimitingEnabled && !speedLimitingOverride && !(maxSpeed == 0)) {
            if (movingTargetPosition < targetPosition) { // move the movingTargetPosition at maxSpeed towards the set targetPosition until the targetPosition is reached
                if (PIDFrameRate > 0) movingTargetPosition += maxSpeed * percentMaxSpeed / PIDFrameRate;
                if (movingTargetPosition > targetPosition) movingTargetPosition = targetPosition;
            } else if (movingTargetPosition > targetPosition) {
                if (PIDFrameRate > 0) movingTargetPosition -= maxSpeed * percentMaxSpeed / PIDFrameRate;
                if (movingTargetPosition < targetPosition) movingTargetPosition = targetPosition;
            } else movingTargetPosition = targetPosition;
        } else movingTargetPosition = targetPosition; // if not speed limiting or maxSpeed is 0, movingTargetPosition just becomes targetPosition
    }


    public double getPower(double overrideCurrentPosition) { // VERY IMPORTANT that this needs to be called constantly to work properly
        // overrideCurrentPosition is just the current encoder Position unless it is being overridden

        if (stoppedUntilNextUse) { // reset all timers and integral when pid is first used again after being stopped
            mRuntime.reset();
            reset();
            stoppedUntilNextUse = false;
        }

        correctValues();
        double power;
        double timeSince = (mRuntime.time() / 1000.0) - previousTime;
        PIDFrameRate = 1.0 / timeSince;

        updateMovingTargetPosition();

        double Error = movingTargetPosition - overrideCurrentPosition; // calculate PID values
        integral += Error * timeSince;
        if (Math.abs(integral) > maxIntegral) integral = Math.signum(integral) * maxIntegral; // stabilize integral
        double Derivative = (Error - lastError) / timeSince;
        lastError = Error;
        previousTime = mRuntime.time() / 1000.0;
        power = (Error * kP) + (integral * kI) + (Derivative * kD); // calculate the result

        power = (Math.abs(power) * (1 - initialPower) + initialPower) * Math.signum(power); // normalizes to start at initial power

        // make sure the power obeys all of the set limits
        if (Math.abs(power) > maxPower) power = Math.signum(power) * maxPower;
        else if (Math.abs(power) < minPower) power = 0.0;

        // prevent motor from continuing to be powered outside of min and max limits if enabled
        if (positionLimitingEnabled) {
            if (overrideCurrentPosition < minPosition && power < 0) power = 0;
            else if (overrideCurrentPosition > maxPosition && power > 0) power = 0;
        }

        return power;
    }


    private void correctValues() { // kind of unnecessary but makes there be less steps when creating code to edit these while driving
        if (kP < 0) kP = 0;
        if (kI < 0) kI = 0;
        if (kD < 0) kD = 0;
        if (minPosition > maxPosition) minPosition = maxPosition;
        if (minPower < 0) minPower = 0;
        if (maxPower > 1) maxPower = 1;
        if (minPower > maxPower) minPower = maxPower;
        if (initialPower < 0) initialPower = 0;
        if (minPower < initialPower) minPower = initialPower; // make sure minPower is at least initialPower
        if (maxSpeed < 0) maxSpeed = 0;
        if (tolerance < 0) tolerance = 0;
        if (kI > 0) maxIntegral = maxPower / kI;
        else maxIntegral = 1;
    }


    public double getPower() { // normal call which just sets current position to current position
        return getPower(encoderPosition.getAsDouble()); // this allows using the PID by setting the targetPosition to 0 and the amount it needs to change by to the overrideCurrentPosition
    }


    public double getPower(double targetPosition, double currentPosition) { // combine setTarget and getPower
        setTarget(targetPosition);
        return getPower(currentPosition);
    }


    // this lets the target position and all positions that are multiples of the target position (ex: 0, 360, 720, 1080, -360)
    // be treated the same so the pid just targets the closest one
    public double getPowerWrapped(double targetPosition, int wrapAmount) {
        // wrapAmount is confusing to explain but if the encoderPosition uses degrees and you want the pid to treat
        // all angles that are equivalent to the target angle the same, wrapAmount would be 360
        return getPower(0.0, angleDifference(encoderPosition.getAsDouble(), targetPosition, wrapAmount));
    }


    // this awesome method is copied here in case anyone wants to use this exact PID class in future code
    private double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }


    public void setPercentMaxSpeed(double PercentMaxSpeed) { // between 0 to 1 and only works when speed limiting is on
        percentMaxSpeed = PercentMaxSpeed;
    }


    public boolean closeEnough() {
        return Math.abs(targetPosition - encoderPosition.getAsDouble()) <= tolerance;
    }


    public void stopUntilNextUse() {
        stoppedUntilNextUse = true;
    }

}
