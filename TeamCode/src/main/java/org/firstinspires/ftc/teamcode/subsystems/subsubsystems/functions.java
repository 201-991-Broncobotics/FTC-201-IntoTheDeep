package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

public class functions {

    // finds the smallest difference between two angles or wraps an angle to between -180 and 180 when target is 0 (when wrapAngle is 360)
    // wrapAngle of 180 treats the targetAngle and the angle opposite of targetAngle the same
    public static double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }


}
