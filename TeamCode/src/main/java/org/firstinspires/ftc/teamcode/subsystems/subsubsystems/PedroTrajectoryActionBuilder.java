package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.commands.DriveAutonCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class PedroTrajectoryActionBuilder {

    private Follower follower;

    private PathBuilder currentPath;

    private Pose lastPose;

    private ArrayList<BooleanSupplier> endConditions = new ArrayList<BooleanSupplier>();

    private double timeOutSeconds;

    private double lastTangent;

    private boolean isReversed = false, OnlyRequireOneEndCondition;


    public PedroTrajectoryActionBuilder(PathBuilder pathConstructor, Pose2d startPose, Follower followerInput) {
        this.currentPath = pathConstructor;
        this.lastPose = functions.RRToPedroPose(startPose);
        this.follower = followerInput;
        OnlyRequireOneEndCondition = true;
        timeOutSeconds = 0;
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

        currentPath.addPath(new BezierPoint(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN)));
        currentPath.setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
        lastTangent = heading;
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), heading);
        return this;
    }


    public PedroTrajectoryActionBuilder strafeTo(Vector2d pos) {
        Point PedroPos = functions.RRToPedroPoint(pos);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos));
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
        lastTangent = Math.atan2(PedroPos.getY() - lastPose.getY(), PedroPos.getX() - lastPose.getX());
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToConstantHeading(Vector2d pos) {
        Point PedroPos = functions.RRToPedroPoint(pos);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos));
        currentPath.setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
        lastTangent = Math.atan2(PedroPos.getY() - lastPose.getY(), PedroPos.getX()- lastPose.getX());
        lastPose = new Pose(PedroPos.getX(), PedroPos.getY(), lastPose.getHeading());
        return this;
    }


    public PedroTrajectoryActionBuilder strafeToLinearHeading(Vector2d pos, double heading) {
        Point PedroPos = functions.RRToPedroPoint(pos);
        heading = functions.RRToPedroAngle(heading);

        currentPath.addPath(new BezierLine(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), PedroPos));
        currentPath.setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
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
        currentPath.setPathEndTimeoutConstraint(1);
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
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), firstPoint, secondPoint, PedroPos));
        currentPath.setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
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
        currentPath.addPath(new BezierCurve(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN), firstPoint, secondPoint, new Point(PedroPose.getX(), PedroPose.getY(), Point.CARTESIAN)));
        currentPath.setLinearHeadingInterpolation(lastPose.getHeading(), PedroPose.getHeading());
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
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
    public PedroTrajectoryActionBuilder bezierTo(Vector2d... controlPoints) {
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Vector2d v : controlPoints) {
            resultArray.add(functions.RRToPedroPoint(v));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve);
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
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
    public PedroTrajectoryActionBuilder bezierToConstantHeading(Vector2d... controlPoints) {
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Vector2d v : controlPoints) {
            resultArray.add(functions.RRToPedroPoint(v));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve);
        currentPath.setConstantHeadingInterpolation(lastPose.getHeading());
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
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
    public PedroTrajectoryActionBuilder bezierToLinearHeading(double heading, Vector2d... controlPoints) {
        heading = functions.RRToPedroAngle(heading);
        ArrayList<Point> resultArray = new ArrayList<Point>();
        for (Vector2d v : controlPoints) {
            resultArray.add(functions.RRToPedroPoint(v));
        }

        ArrayList<Point> bezierCurvePoints = new ArrayList<Point>();
        bezierCurvePoints.add(new Point(lastPose.getX(), lastPose.getY(), Point.CARTESIAN));
        bezierCurvePoints.addAll(resultArray);
        BezierCurve actualBezierCurve = new BezierCurve(bezierCurvePoints);
        currentPath.addPath(actualBezierCurve);
        currentPath.setLinearHeadingInterpolation(lastPose.getHeading(), heading);
        currentPath.setReversed(isReversed);
        currentPath.setPathEndTimeoutConstraint(1);
        lastTangent = actualBezierCurve.getEndTangent().getTheta();
        Point endControlPoint = actualBezierCurve.getLastControlPoint();
        lastPose = new Pose(endControlPoint.getX(), endControlPoint.getY(), heading);
        return this;
    }




    // PATH CONSTRAINTS - applies to the last path segment and only applies to the final end position of it

    /**
     * This sets the velocity stop criteria. When velocity is below this amount, then this is met.
     */
    public PedroTrajectoryActionBuilder setPathEndVelocityConstraint(double velocity) {
        currentPath.setPathEndVelocityConstraint(velocity);
        return this;
    }
    /**
     * This sets the translational stop criteria. When the translational error, or how far off the
     * end point the robot is, goes below this, then the translational end criteria is met.
     */
    public PedroTrajectoryActionBuilder setPathEndTranslationalConstraint(double distance) {
        currentPath.setPathEndTranslationalConstraint(distance);
        return this;
    }
    /**
     * This sets the heading stop criteria. When the heading error is less than this amount, then
     * the heading end criteria is met.
     */
    public PedroTrajectoryActionBuilder setPathEndHeadingConstraint(double degrees) {
        currentPath.setPathEndHeadingConstraint(degrees);
        return this;
    }
    /**
     * This sets the parametric end criteria. When the t-value of the closest Point on the Path is
     * greater than this amount, then the parametric end criteria is met.
     */
    public PedroTrajectoryActionBuilder setPathEndTValueConstraint(double parametricTime) {
        currentPath.setPathEndTValueConstraint(parametricTime);
        return this;
    }
    /**
     * This sets the Path end timeout. If the Path is at its end parametrically, then the Follower
     * has this many milliseconds to correct before the Path gets ended anyways.
     */
    public PedroTrajectoryActionBuilder setPathEndTimeoutConstraint(double milliseconds) {
        currentPath.setPathEndTimeoutConstraint(milliseconds);
        return this;
    }






    // CALLBACKS - or stuff that run other code when a condition is met on the path

    /**
     * This is just to convert whatever an instantFunction is to a Roadrunner Action
     */
    private Action InstantAction(InstantFunction f) { // Thank God that Android Studio can auto build most of this function
        return telemetryPacket -> {
            f.run();
            return false;
        };
    }

    /**
     * This runs Pedro's parametric callback: This adds a parametric callback on the last Path added to the PathBuilder.
     * This callback is set to run at a certain point on the Path.
     * @param ds between 0 and 1 where 0.995 is the usually definition of the parametric end (the end of the path)
     * @param a The Action that needs to be run
     */
    public PedroTrajectoryActionBuilder afterDisp(double ds, Action a) {
        currentPath.addParametricCallback(ds, DriveAutonCommand.queueActionRunnable(a));
        return this;
    }
    /**
     * This runs Pedro's parametric callback: This adds a parametric callback on the last Path added to the PathBuilder.
     * This callback is set to run at a certain point on the Path.
     * @param ds between 0 and 1 where 0.995 is the usually definition of the parametric end (the end of the path)
     * @param f The InstantFunction that needs to be run
     */
    public PedroTrajectoryActionBuilder afterDisp(double ds, InstantFunction f) {
        return afterDisp(ds, InstantAction(f));
    }

    public PedroTrajectoryActionBuilder afterTime(double dt, Action a) {
        currentPath.addTemporalCallback(dt, DriveAutonCommand.queueActionRunnable(a));
        return this;
    }
    public PedroTrajectoryActionBuilder afterTime(double dt, InstantFunction f) {
        return afterTime(dt, InstantAction(f));
    }





    // END CONDITIONS

    /**
     * This end condition is true when Pedro has stopped following the path because it has finished
     * or been ended.
     */
    public PedroTrajectoryActionBuilder endWhenDoneFollowing() { // this is the default end condition when no other condition is given
        endConditions.add(() -> !follower.isBusy());
        return this;
    }
    /**
     * This end condition applies when the robot is at the end of its path.
     */
    public PedroTrajectoryActionBuilder endWhenAtFinish() {
        endConditions.add(() -> follower.atParametricEnd());
        return this;
    }
    /**
     * This end condition makes it so the robot path will end only once the robot has stopped moving
     * at the end of its path.
     */
    public PedroTrajectoryActionBuilder endWhenStoppedMovingAtFinish() {
        endConditions.add(() -> follower.atParametricEnd() && follower.getVelocityMagnitude() < 1);
        return this;
    }
    /**
     * This end condition makes it so the robot path will end only once the robot is moving at less
     * than 1 inch per second.
     */
    public PedroTrajectoryActionBuilder endWhenStoppedMoving() {
        endConditions.add(() -> follower.getVelocityMagnitude() < 1);
        return this;
    }
    /**
     * This end condition makes it so the robot path will end only once the robot is moving at less
     * than 1 inch per second and has past the half way point through the path.
     */
    public PedroTrajectoryActionBuilder endWhenStoppedMovingPastHalfWay() {
        endConditions.add(() -> follower.getVelocityMagnitude() < 1 && follower.pastParametricPoint(0.5));
        return this;
    }
    /**
     * This end condition makes the robot path end once the robot has arrived within a certain distance
     * away from the end of the path.
     * @param distanceFromFinish in inches
     */
    boolean endWhenWithinRangeOfFinishIsWaitingForThePathToBeCompleted = false; // yes
    double distanceFromFinishThatPathIsEnded = 2;
    public PedroTrajectoryActionBuilder endWhenWithinRangeOfFinish(double distanceFromFinish) {
        endWhenWithinRangeOfFinishIsWaitingForThePathToBeCompleted = true;
        distanceFromFinishThatPathIsEnded = distanceFromFinish;
        return this;
    }
    /**
     * This end condition makes it so the robot will stop following if it has been taking too long.
     * @param seconds length of time
     */
    public PedroTrajectoryActionBuilder endAfterTimeout(double seconds) {
        // endConditions.add(() -> follower.getFollowingRuntime() > seconds);
        timeOutSeconds = seconds;
        return this;
    }
    /**
     * This end condition makes it so the robot will stop following if it has been taking too long.
     * @param seconds length of time
     */
    private PedroTrajectoryActionBuilder endAfterTimeSincePathFinished(double seconds) { // TODO: may not work
        endConditions.add(() -> follower.getTimeSincePathFinished() > seconds);
        return this;
    }



    public PedroTrajectoryActionBuilder requireAllEndConditions() {
        OnlyRequireOneEndCondition = false;
        return this;
    }


    /**
     * This returns the path constructed so far though it doesn't include the custom end conditions.
     * This allows you to run one of these paths in TeleOp if you give this to the follower.
     * @return PathChain
     */
    public PathChain getPath() {
        return currentPath.setPathEndTimeoutConstraint(FollowerConstants.pathEndTimeoutConstraint).build();
    }

    /**
     * This returns the pose of where the path ends
     * @return Pose2d
     */
    public Pose2d endPose() {
        return functions.PedroToRRPose(lastPose);
    }

    public Action build() {
        if (endWhenWithinRangeOfFinishIsWaitingForThePathToBeCompleted) {
            endConditions.add(() -> (Math.abs(follower.getPose().getX() - lastPose.getX()) > distanceFromFinishThatPathIsEnded || Math.abs(follower.getPose().getY() - lastPose.getY()) > distanceFromFinishThatPathIsEnded));
        }

        if (endConditions.isEmpty()) {
            endWhenDoneFollowing(); // default end condition
        }
        return follower.followPathAction(currentPath.build(), endConditions, OnlyRequireOneEndCondition, timeOutSeconds);
    }


}
