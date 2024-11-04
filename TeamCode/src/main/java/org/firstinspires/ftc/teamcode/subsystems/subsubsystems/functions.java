package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Constants;

public class functions {

    // finds the smallest difference between two angles or wraps an angle to between -180 and 180 when target is 0 (when wrapAngle is 360)
    // wrapAngle of 180 treats the targetAngle and the angle opposite of targetAngle the same
    public static double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }

    public static double round(double input, int decimalPlaces) {
        return Math.round(input * (Math.pow(10, decimalPlaces))) / (Math.pow(10, decimalPlaces));
    }

    public static boolean inUse(double value) {
        return (Math.abs(value) > Constants.controllerDeadZone);
    }

    public static void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds
        }
    }

    public static Vector2d tileCoords(double X, double Y) { // convert field coords in tiles to coords in inches
        return new Vector2d(X / Constants.tileLength, Y / Constants.tileLength);
    }


    public static double tiles(double tiles) { // convert number of tiles to inches
        return tiles * Constants.tileLength;
    }

    public static double capValue(double input, double maxValue) {
        if (Math.abs(input) > maxValue) input = maxValue * Math.signum(input);
        return input;
    }


}
