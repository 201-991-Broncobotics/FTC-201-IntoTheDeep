package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import static org.firstinspires.ftc.teamcode.SubsystemData.log;
import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.addAnglesRad;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;

public class PedroTrajectoryActionBuilder {

    Follower follower;

    PathBuilder currentPath;

    Pose lastPose;

    double lastTangent;

    boolean isReversed = false;


    public PedroTrajectoryActionBuilder(PathBuilder pathConstructor, Pose2d startPose, Follower followerInput) {
        this.currentPath = pathConstructor;
        this.lastPose = functions.RRToPedroPose(startPose);
        this.follower = followerInput;
    }


    public PedroTrajectoryActionBuilder setTangent(double Rotation) {
        lastTangent = functions.RRToPedroAngle(Rotation);
        return this;
    }


    public PedroTrajectoryActionBuilder setReversed(boolean reversed) {
        isReversed = reversed;
        return this;
    }


    public PedroTrajectoryActionBuilder turnTo(double heading) {
        heading = functions.RRToPedroAngle(heading);

        currentPath.addPath(new BezierPoint(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        lastTangent = heading;
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), heading);
        return this;
    }


    public PedroTrajectoryActionBuilder strafeTo(Vector2d pos) {
        Point PedroPos = functions.RRToPedroPoint(pos);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos));
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(PedroPos.getY() - lastPose.getY(), PedroPos.getX() - lastPose.getX());
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToConstantHeading(Vector2d pos) {
        Point PedroPos = functions.RRToPedroPoint(pos);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos))
                .setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(PedroPos.getY() - lastPose.getY(), PedroPos.getX()- lastPose.getX());
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToLinearHeading(Vector2d pos, double heading) {
        Point PedroPos = functions.RRToPedroPoint(pos);
        heading = functions.RRToPedroAngle(heading);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos))
                .setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(PedroPos.getY() - lastPose.getY(), PedroPos.getX() - lastPose.getX());
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), heading);
        return this;
    }


    public PedroTrajectoryActionBuilder splineTo(Vector2d pos, double tangent) {
        Point PedroPos = functions.RRToPedroPoint(pos);
        tangent = functions.RRToPedroAngle(tangent);

        double tangentDistance = 0.429412 * Math.hypot(PedroPos.getX() - lastPose.getX(), PedroPos.getY() - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent), Point.CARTESIAN);
        Point secondPoint = new Point(PedroPos.getX() + tangentDistance * Math.cos(tangent + Math.PI), PedroPos.getY() + tangentDistance * Math.sin(tangent + Math.PI), Point.CARTESIAN);
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), firstPoint, secondPoint, PedroPos));
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder splineToConstantHeading(Vector2d pos, double tangent) {
        Point PedroPos = functions.RRToPedroPoint(pos);
        tangent = functions.RRToPedroAngle(tangent);

        double tangentDistance = 0.429412 * Math.hypot(PedroPos.getX() - lastPose.getX(), PedroPos.getY() - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent), Point.CARTESIAN);
        Point secondPoint = new Point(PedroPos.getX() + tangentDistance * Math.cos(tangent + Math.PI), PedroPos.getY() + tangentDistance * Math.sin(tangent + Math.PI), Point.CARTESIAN);
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), firstPoint, secondPoint, PedroPos))
                .setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }

    public PedroTrajectoryActionBuilder splineToLinearHeading(Pose2d pose, double tangent) {
        Pose PedroPose = functions.RRToPedroPose(pose);
        tangent = functions.RRToPedroAngle(tangent);

        double tangentDistance = 0.429412 * Math.hypot(PedroPose.getX() - lastPose.getX(), PedroPose.getY() - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent), Point.CARTESIAN);
        Point secondPoint = new Point(PedroPose.getX() + tangentDistance * Math.cos(tangent + Math.PI), PedroPose.getY() + tangentDistance * Math.sin(tangent + Math.PI), Point.CARTESIAN);
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), firstPoint, secondPoint, new Point(PedroPose.getX(), PedroPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(lastPose.getHeading(), PedroPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = PedroPose;
        return this;
    }


    // NON ROADRUNNER METHODS - MeepMeep can't draw these out

    /**
     * This will allow the robot to follow a bezier curve like it was part of a roadrunner trajectory.
     * @param controlPoints This is the specified control points that define the BezierCurve. NOTE:
     *                      Do NOT put the start point as that is assumed to be at the end of the
     *                      last path.
     */
    public PedroTrajectoryActionBuilder bezierTo(Point... controlPoints) {
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Point i : controlPoints) {
            resultArray.add(new Point(i.getY(), -1 * i.getX(), Point.CARTESIAN));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve);
        currentPath.setReversed(isReversed);
        lastTangent = actualBezierCurve.getEndTangent().getTheta();
        Point endControlPoint = actualBezierCurve.getLastControlPoint();
        lastPose = new Pose(endControlPoint.getX(), endControlPoint.getY(), lastPose.getHeading());
        return this;
    }


    /**
     * This will allow the robot to follow a bezier curve like it was part of a roadrunner trajectory.
     * @param controlPoints This is the specified control points that define the BezierCurve. NOTE:
     *                      Do NOT put the start point as that is assumed to be at the end of the
     *                      last path.
     */
    public PedroTrajectoryActionBuilder bezierToConstantHeading(Point... controlPoints) {
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Point i : controlPoints) {
            resultArray.add(new Point(i.getY(), -1 * i.getX(), Point.CARTESIAN));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve)
                .setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = actualBezierCurve.getEndTangent().getTheta();
        Point endControlPoint = actualBezierCurve.getLastControlPoint();
        lastPose = new Pose(endControlPoint.getX(), endControlPoint.getY(), lastPose.getHeading());
        return this;
    }


    /**
     * This will allow the robot to follow a bezier curve like it was part of a roadrunner trajectory.
     * @param heading This is the target heading
     * @param controlPoints This is the specified control points that define the BezierCurve. NOTE:
     *                      Do NOT put the start point as that is already assumed to be at the end of
     *                      the last path.
     */
    public PedroTrajectoryActionBuilder bezierToLinearHeading(double heading, Point... controlPoints) {
        heading = functions.RRToPedroAngle(heading);
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Point i : controlPoints) {
            resultArray.add(new Point(i.getY(), -1 * i.getX(), Point.CARTESIAN));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve)
                .setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        lastTangent = actualBezierCurve.getEndTangent().getTheta();
        Point endControlPoint = actualBezierCurve.getLastControlPoint();
        lastPose = new Pose(endControlPoint.getX(), endControlPoint.getY(), heading);
        return this;
    }


    /**
     * This returns the pose of where the path ends
     * @return Pose2d
     */
    public Pose2d endPose() {
        return functions.PedroToRRPose(lastPose);
    }


    public Action build() {
        return follower.followPathAction(currentPath.build(), lastPose);
    }


}
