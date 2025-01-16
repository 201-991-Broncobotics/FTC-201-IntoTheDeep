package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public class functions {

    // finds the smallest difference between two angles or gets the equivalent angle between -180 and 180 when the currentAngle is 0 (and wrapAngle is 360)
    // or finds the equivalent angle between 0 and 360 when the currentAngle is -180 and you add +180 to the result
    // wrapAngle of 180 treats the targetAngle and the angle directly opposite of targetAngle the same
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

    public static double deadZone(double value) {
        if (Math.abs(value) > Constants.controllerDeadZone) return value;
        else return 0.0;
    }

    public static void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds
        }
    }

    public static Vector2d tileCoords(double X, double Y) { // convert field coords in tiles to coords in inches
        return new Vector2d(X * Constants.tileLength, Y * Constants.tileLength);
    }


    public static Point tileCoordsP(double X, double Y) {
        return new Point(X * Constants.tileLength, Y * Constants.tileLength);
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


}
