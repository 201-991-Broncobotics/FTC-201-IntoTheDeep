package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Settings;



public class functions {

    /** Finds the smallest difference between two angles or gets the equivalent angle between -180 and
     * 180 when the currentAngle is 0 (and wrapAngle is 360).
     * A wrapAngle of 180 treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle degrees
     * @param targetAngle degrees
     * @param wrapAngle Should be 360 unless you are doing a swerve module
     */
    public static double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }

    public static double normalizeAngle(double angle) {
        return angleDifference(-180, angle, 360) + 180;
    }

    public static double round(double input, int decimalPlaces) {
        return Math.round(input * (Math.pow(10, decimalPlaces))) / (Math.pow(10, decimalPlaces));
    }

    public static boolean inUse(double value) {
        return (Math.abs(value) > Settings.controllerDeadZone);
    }

    public static double deadZone(double value) {
        if (Math.abs(value) > Settings.controllerDeadZone) return value;
        else return 0.0;
    }

    public static double capValue(double value, double limit) {
        if (Math.abs(value) > limit) return limit * Math.signum(value);
        else return value;
    }

    /**
     * If the absolute value of the input is less than the deadZone, this returns 0 though if it is greater,
     * this returns the difference between the deadZone and the input. Example: -0.5, -0.25, 0, 0, 0, 0.25, 0.5
     */
    public static double deadZoneFlattened(double value, double deadZone) {
        if (Math.abs(value) > deadZone) return value - Math.abs(deadZone) * Math.signum(value);
        else return 0.0;
    }

    public static double deadZoneNormalized(double value, double deadZone) {
        return deadZoneFlattened(value, deadZone) * Math.abs(1 / (1 - deadZone));
    }

    public static void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds
        }
    }

    /**
     * Converts Coordinates based on the number of field tiles to inches in the Roadrunner coordinate system
     * @param X tiles (23.5625 inches)
     * @param Y tiles (23.5625 inches)
     * @return inches
     */
    public static Vector2d tileCoords(double X, double Y) { // convert field coords in tiles to coords in inches
        return new Vector2d(X * Constants.tileLength, Y * Constants.tileLength);
    }

    public static double tiles(double tiles) { // convert number of tiles to inches
        return tiles * Constants.tileLength;
    }

    public static boolean intListContains(int[] list, int value) {
        for (int item : list) {
            if (item == value) return true;
        }
        return false;
    }

    public static double addAnglesRad(double angleRadians1, double angleRadians2) {
        double sum = angleRadians1 + angleRadians2;
        return Math.toRadians(angleDifference(-180, Math.toDegrees(sum), 360) + 180);
    }

    public static double addAnglesDeg(double angleDegrees1, double angleDegrees2) {
        double sum = angleDegrees1 + angleDegrees2;
        return angleDifference(-180, sum, 360) + 180;
    }

    public static Pose RRToPedroPose(Pose2d pose) {
        return new Pose(pose.position.y, -1 * pose.position.x, addAnglesRad(pose.heading.toDouble(), Math.PI/-2));
    }

    public static Pose2d PedroToRRPose(Pose pose) {
        return new Pose2d(new Vector2d(-1 * pose.getY(), pose.getX()), addAnglesRad(pose.getHeading(), Math.PI/2));
    }

    public static Point RRToPedroPoint(Vector2d vector) {
        return new Point(vector.y, -1 * vector.x, Point.CARTESIAN);
    }

    public static Vector2d PedroToRRPoint(Point point) {
        return new Vector2d(-1 * point.getY(), point.getX());
    }

    public static double RRToPedroAngle(double angle) {
        return addAnglesRad(angle, Math.PI/-2);
    }

    public static double PedroToRRAngle(double angle) {
        return addAnglesRad(angle, Math.PI/2);
    }

    public static String PoseAsString(Pose pose) {
        return "X:" + round(pose.getX(), 2) + " Y:" + round(pose.getY(), 2) + " H:" + round(Math.toDegrees(pose.getHeading()), 2);
    }

    public static String PoseAsString(Pose2d pose) {
        return "X:" + round(pose.position.x, 2) + " Y:" + round(pose.position.y, 2) + " H:" + round(Math.toDegrees(pose.heading.toDouble()), 2);
    }

    public static String TilePoseAsString(Pose pose) {
        return "X:" + round(tiles(pose.getX()), 3) + " Y:" + round(tiles(pose.getY()), 3) + " H:" + round(Math.toDegrees(pose.getHeading()), 2);
    }

    public static String TilePoseAsString(Pose2d pose) {
        return "X:" + round(tiles(pose.position.x), 3) + " Y:" + round(tiles(pose.position.y), 3) + " H:" + round(Math.toDegrees(pose.heading.toDouble()), 2);
    }


    public static boolean isPointAboveLine(Vector2d point, Vector2d linePoint1, Vector2d linePoint2) {
        return (((linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x)
                * (point.x - linePoint1.x) + linePoint1.y) - point.y) <= 0;
    }


    public static double minMaxValue(double min, double max, double value) {
        if (value > max) return max;
        else return Math.max(value, min);
    }

}
