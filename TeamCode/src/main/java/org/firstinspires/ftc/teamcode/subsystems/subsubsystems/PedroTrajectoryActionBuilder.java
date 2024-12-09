package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

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
        this.lastPose = new Pose(startPose.position.x, startPose.position.y, startPose.heading.toDouble());
        this.follower = followerInput;
    }


    public PedroTrajectoryActionBuilder setTangent(double Rotation) {
        lastTangent = Rotation;
        return this;
    }


    public PedroTrajectoryActionBuilder setReversed(boolean reversed) {
        isReversed = reversed;
        return this;
    }


    public PedroTrajectoryActionBuilder turnTo(double heading) {
        currentPath.addPath(new BezierPoint(new Point(lastPose.getX(), lastPose.getY())))
                .setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        lastTangent = heading;
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), heading);
        return this;
    }


    public PedroTrajectoryActionBuilder strafeTo(Vector2d pos) {
        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY()), new Point(pos.x, pos.y)));
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(pos.y - lastPose.getY(), pos.x - lastPose.getX());
        lastPose = new Pose(pos.x, pos.y, lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToConstantHeading(Vector2d pos) {
        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY()), new Point(pos.x, pos.y)))
                .setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(pos.y - lastPose.getY(), pos.x - lastPose.getX());
        lastPose = new Pose(pos.x, pos.y, lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToLinearHeading(Vector2d pos, double heading) {
        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY()), new Point(pos.x, pos.y)))
                .setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        lastTangent = Math.atan2(pos.y - lastPose.getY(), pos.x - lastPose.getX());
        lastPose = new Pose(pos.x, pos.y, heading);
        return this;
    }


    public PedroTrajectoryActionBuilder splineTo(Vector2d pos, double tangent) {
        double tangentDistance = 0.429412 * Math.hypot(pos.x - lastPose.getX(), pos.y - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent));
        Point secondPoint = new Point(pos.x + tangentDistance * Math.cos(tangent + Math.PI), pos.y + tangentDistance * Math.sin(tangent + Math.PI));
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY()), firstPoint, secondPoint, new Point(pos.x, pos.y)));
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = new Pose(pos.x, pos.y, lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder splineToConstantHeading(Vector2d pos, double tangent) {
        double tangentDistance = 0.429412 * Math.hypot(pos.x - lastPose.getX(), pos.y - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent));
        Point secondPoint = new Point(pos.x + tangentDistance * Math.cos(tangent + Math.PI), pos.y + tangentDistance * Math.sin(tangent + Math.PI));
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY()), firstPoint, secondPoint, new Point(pos.x, pos.y)))
                .setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = new Pose(pos.x, pos.y, lastPose.getHeading());
        return this;
    }

    public PedroTrajectoryActionBuilder splineToLinearHeading(Pose2d pose, double tangent) {
        double tangentDistance = 0.429412 * Math.hypot(pose.position.x - lastPose.getX(), pose.position.y - lastPose.getY());
        Point firstPoint = new Point(lastPose.getX() + tangentDistance * Math.cos(lastTangent), lastPose.getY() + tangentDistance * Math.sin(lastTangent));
        Point secondPoint = new Point(pose.position.x + tangentDistance * Math.cos(tangent + Math.PI), pose.position.y + tangentDistance * Math.sin(tangent + Math.PI));
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY()), firstPoint, secondPoint, new Point(pose.position.x, pose.position.y)))
                .setLinearHeadingInterpolation(lastPose.getHeading(), pose.heading.toDouble());
        currentPath.setReversed(isReversed);
        lastTangent = tangent;
        lastPose = new Pose(pose.position.x, pose.position.y, pose.heading.toDouble());
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
        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY()));
        bezierCurvePoints.addAll(Arrays.asList(controlPoints));
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
        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY()));
        bezierCurvePoints.addAll(Arrays.asList(controlPoints));
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
     *                      Do NOT put the start point as that is assumed to be at the end of the
     *                      last path.
     */
    public PedroTrajectoryActionBuilder bezierToLinearHeading(double heading, Point... controlPoints) {
        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY()));
        bezierCurvePoints.addAll(Arrays.asList(controlPoints));
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
        return new Pose2d(lastPose.getX(), lastPose.getY(), lastPose.getHeading());
    }


    public Action build() {
        return follower.followPathAction(currentPath.build(), lastPose);
    }


}
