package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions.addAnglesRad;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.Roadrunner.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

/**
 * This is the RRToPedroThreeWheelLocalizer class. This class extends the Localizer superclass and
 * is intended to adapt the working Road Runner two wheel odometry localizer to the stupid Pedro Pathing
 * localizer system.
 *
 * @author Aidan Turico - 201 - Im actually the author now haha
 * @version 1.0, 12/7/2024
 */
public class RRToPedroTwoWheelLocalizer extends Localizer {
    private double totalHeading;

    DifferentialSwerveDrive drive;
    private Pose startPose;
    private Pose previousPose;

    /**
     * This creates a new RRToPedroThreeWheelLocalizer from a HardwareMap. This adapts the previously
     * used Road Runner localization system to the new Pedro Pathing localization system.
     *
     * @param hardwareMap the HardwareMap
     */
    public RRToPedroTwoWheelLocalizer(HardwareMap hardwareMap, DifferentialSwerveDrive RRLocalizer) {
        drive = RRLocalizer;

        startPose = new Pose();
        previousPose = new Pose();
    }

    /**
     * This returns the current pose estimate as a Pose.
     *
     * @return returns the current pose estimate
     */
    @Override
    public Pose getPose() {
        Pose2d pose = drive.pose;
        return new Pose(pose.position.y, -1 * pose.position.x, addAnglesRad(pose.heading.toDouble(), Math.PI/-2));
    }

    /**
     * This returns the current velocity estimate as a Pose.
     *
     * @return returns the current velocity estimate
     */
    @Override
    public Pose getVelocity() {
        PoseVelocity2d poseVel = SubsystemData.RobotVelocity;
        return new Pose(poseVel.linearVel.x, poseVel.linearVel.y, poseVel.angVel);
    }

    /**
     * This returns the current velocity estimate as a Vector.
     *
     * @return returns the current velocity estimate
     */
    @Override
    public Vector getVelocityVector() {
        PoseVelocity2d poseVel = SubsystemData.RobotVelocity;
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(poseVel.linearVel.x, poseVel.linearVel.y);
        return returnVector;
    }

    /**
     * This sets the start pose. Any movement of the robot is treated as a displacement from the
     * start pose, so moving the start pose will move the current pose estimate the same amount.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        Pose oldStart = startPose;
        startPose = setStart;
        Pose startDiff = MathFunctions.subtractPoses(startPose, oldStart);
        drive.pose = functions.PedroToRRPose(new Pose(getPose().getX() + startDiff.getX(), getPose().getY() + startDiff.getY(), getPose().getHeading() + startDiff.getHeading()));
    }

    /**
     * This sets the current pose estimate. This has no effect on the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        drive.pose = new Pose2d(-1 * setPose.getY(), setPose.getX(), addAnglesRad(setPose.getHeading(), Math.PI/2));
    }

    /**
     * This updates the total heading and previous pose estimate. Everything else is handled by the
     * Road Runner localizer on its own, but updating this tells you how far the robot has really
     * turned.
     */
    @Override
    public void update() {
        drive.updatePoseEstimate();
        totalHeading += MathFunctions.getTurnDirection(previousPose.getHeading(), getPose().getHeading()) * MathFunctions.getSmallestAngleDifference(previousPose.getHeading(), getPose().getHeading());
        previousPose = getPose();
    }

    /**
     * This returns how far the robot has actually turned.
     *
     * @return returns the total angle turned, in degrees.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the forward multiplier of the Road Runner localizer, which converts from ticks
     * to inches. You can actually use the tuners in Pedro Pathing to find the value that everything
     * multiplied together should be. If you do use that, then do be aware that the value returned is
     * the product of the Road Runner ticks to inches and the x multiplier.
     *
     * @return returns the forward multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return DifferentialSwerveDrive.PARAMS.inPerTick;
    }

    /**
     * This returns the lateral multiplier of the Road Runner localizer, which converts from ticks
     * to inches. You can actually use the tuners in Pedro Pathing to find the value that everything
     * multiplied together should be. If you do use that, then do be aware that the value returned is
     * the product of the Road Runner ticks to inches and the y multiplier.
     *
     * @return returns the lateral multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return DifferentialSwerveDrive.PARAMS.lateralInPerTick;
    }

    /**
     * This returns the turning multiplier of the Road Runner localizer, which doesn't actually exist.
     * There really isn't a point in tuning the turning for the Road Runner localizer. This will
     * actually just return the average of the two other multipliers.
     *
     * @return returns the turning multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return (getForwardMultiplier() + getLateralMultiplier()) / 2;
    }
    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() throws InterruptedException {
        drive.realignHeading();
    };
}
