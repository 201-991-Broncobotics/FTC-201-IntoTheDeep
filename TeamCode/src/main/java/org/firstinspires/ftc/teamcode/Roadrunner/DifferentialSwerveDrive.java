package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.Roadrunner.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.Roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DiffySwerveKinematics;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.PersistentDataStorage;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.functions;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class DifferentialSwerveDrive extends SubsystemBase { // This used to be Roadrunner's MecanumDrive

    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = (32 / 25.4 / 2000) * (140.5 / 40.3852); // should be the first term but isn't for some reason
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 341.4816515824069;

        // feedforward parameters (in tick units)
        public double kS = 0.5; // 3.7576427826935133
        public double kV = 0.0019895240817372076;
        public double kA = 2.0; // 2.0 is what this needs to be to match the peaks of v0 and vf

        // path profile parameters (in inches)
        public double maxWheelVel = 20;
        public double minProfileAccel = -15;
        public double maxProfileAccel = 15;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI / 3; // shared with path
        public double maxAngAccel = Math.PI / 3;

        // path controller gains
        public double axialGain = 0.0; // im using my own pids cause these suck
        public double lateralGain = axialGain;
        public double headingGain = 0.0;

        public double axialVelGain = 0.0;
        public double lateralVelGain = axialVelGain;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();


    Telemetry telemetry;

    IMU newImu;

    public static DiffySwerveKinematics diffySwerve;

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(DifferentialSwerveDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(DifferentialSwerveDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(DifferentialSwerveDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(DifferentialSwerveDrive.this.rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public DifferentialSwerveDrive(HardwareMap hardwareMap, Pose2d pose, Telemetry newTelemetry) {
        this.pose = pose;

        telemetry = newTelemetry;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "R3");
        leftBack = hardwareMap.get(DcMotorEx.class, "R4");
        rightBack = hardwareMap.get(DcMotorEx.class, "R1");
        rightFront = hardwareMap.get(DcMotorEx.class, "R2");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD); // this needs to be reversed while tuning roadrunner
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD); // this needs to be reversed while tuning roadrunner


        // telemetry.addData("Left Module Angle Before Reset:", functions.angleDifference((leftFront.getCurrentPosition() / Constants.encoderResolution * 360), 0, 360));
        // telemetry.addData("Right Module Angle Before Reset:", functions.angleDifference((rightFront.getCurrentPosition() / Constants.encoderResolution * 360), 0, 360));


        //leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // 0.012, 0.0, 0.0005

        // 0.05, 0.0, 0.0015

        // Custom Heading PID controller for auton as roadrunner's doesn't work how I want it to
        SubsystemData.HeadingTargetPID = new PIDController(0.012, 0.0, 0.0005, () -> Math.toDegrees(this.pose.heading.toDouble()));
        SubsystemData.HeadingTargetPID.minDifference = 0.5; // helps get the drivetrain turning when at low values

        SubsystemData.AxialPID = new PIDController(0.3, 0.0, 0.0, () -> this.pose.position.y);
        SubsystemData.LateralPID = new PIDController(SubsystemData.AxialPID.kP, SubsystemData.AxialPID.kI, SubsystemData.AxialPID.kD, () -> this.pose.position.x);
        SubsystemData.AxialPID.minDifference = 0.5;
        SubsystemData.LateralPID.minDifference = 0.5;


        SubsystemData.IMUWorking = true;

        SubsystemData.brokenDiffyEncoder = rightFront;

        // secretly initializing diffy alongside mecanum...

        diffySwerve = new DiffySwerveKinematics(leftFront, leftBack, rightBack, rightFront, Constants.maxDrivetrainMotorPower, telemetry);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html

        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        newImu = lazyImu.get();
        SubsystemData.imuInstance = newImu;

        localizer = new TwoDeadWheelLocalizer(hardwareMap, newImu, PARAMS.inPerTick);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        PoseVelocity2dDual<Time> TheCommand = PoseVelocity2dDual.constant(powers, 1);

        diffySwerve.driveDifferentialSwerve(TheCommand.linearVel.y.get(0), TheCommand.linearVel.x.get(0), TheCommand.angVel.get(0), SubsystemData.driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        //if (SubsystemData.DriveMotorHighCurrents[0] < leftFront.getCurrent(CurrentUnit.AMPS)) SubsystemData.DriveMotorHighCurrents[0] = leftFront.getCurrent(CurrentUnit.AMPS);
        //if (SubsystemData.DriveMotorHighCurrents[1] < leftBack.getCurrent(CurrentUnit.AMPS)) SubsystemData.DriveMotorHighCurrents[1] = leftBack.getCurrent(CurrentUnit.AMPS);
        //if (SubsystemData.DriveMotorHighCurrents[2] < rightBack.getCurrent(CurrentUnit.AMPS)) SubsystemData.DriveMotorHighCurrents[2] = rightBack.getCurrent(CurrentUnit.AMPS);
        //if (SubsystemData.DriveMotorHighCurrents[3] < rightFront.getCurrent(CurrentUnit.AMPS)) SubsystemData.DriveMotorHighCurrents[3] = rightFront.getCurrent(CurrentUnit.AMPS);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                diffySwerve.stopDifferentialSwerve();
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();


            txWorldTarget = OffsetTargetPositionWithVel(txWorldTarget, robotVelRobot, SubsystemData.targetPosePerpOffset);


            double txHeadingDouble = Math.atan2(txWorldTarget.heading.imag.value(), txWorldTarget.heading.real.value());
            SubsystemData.AutonError = new Pose2d(new Vector2d(txWorldTarget.position.x.value() - pose.position.x, txWorldTarget.position.y.value() - pose.position.y), txHeadingDouble - pose.heading.toDouble());

            SubsystemData.LateralPID.setVariablesTheSameAs(SubsystemData.AxialPID); // makes sure both PIDs have the same settings
            SubsystemData.AxialPID.setTarget(txWorldTarget.position.y.value());
            SubsystemData.LateralPID.setTarget(txWorldTarget.position.x.value());

            //PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>( // create my own command with proper pid values
                    //new DualNum<Time>(new double[] {SubsystemData.LateralPID.getPower(pose.position.x), 1}),
                    //new DualNum<Time>(new double[] {SubsystemData.AxialPID.getPower(pose.position.y), 1})),
                    //new DualNum<Time>(new double[] {-1 * SubsystemData.HeadingTargetPID.getPowerWrapped(Math.toDegrees(txHeadingDouble), 360), 1}));

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>( // create my own command with proper pid values
                    new DualNum<Time>(new double[] {SubsystemData.AutonError.position.x * SubsystemData.AutonMovementGain, 1}),
                    new DualNum<Time>(new double[] {SubsystemData.AutonError.position.y * SubsystemData.AutonMovementGain, 1})),
                    new DualNum<Time>(new double[] {-1 * SubsystemData.HeadingTargetPID.getPowerWrapped(Math.toDegrees(txHeadingDouble), 360), 1}));

            // for telemetry and roadrunner's dashboard
            driveCommandWriter.write(new DriveCommandMessage(command));

            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            telemetry.addLine("Target Pose X:" + functions.round(txWorldTarget.position.x.get(0), 3) + " Y:" + functions.round(txWorldTarget.position.y.get(0), 3) + " A:" + functions.round(txHeadingDouble, 3));

            // telemetry.addData("Error Distance:", Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y));

            if (Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y) < SubsystemData.AutonStoppingDistance) {
                command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>(
                        new DualNum<Time>(new double[] {0, command.linearVel.x.get(1)}),
                        new DualNum<Time>(new double[] {0, command.linearVel.y.get(1)})),
                        command.angVel);
            }
            if (Math.abs(SubsystemData.AutonError.heading.toDouble()) < SubsystemData.AutonAngleStoppingDifference) {
                command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>(
                        command.linearVel.x,
                        command.linearVel.y),
                        new DualNum<Time>(new double[] {0, command.angVel.get(1)}));
            }

            telemetry.addLine("command:" + functions.round(t, 3) + " X:" + functions.round(command.linearVel.x.value(), 3) + " Y:" + functions.round(command.linearVel.y.value(), 3) + " A:" + functions.round(command.angVel.value(), 3));

            if (Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y) < SubsystemData.AutonStoppingDistance && Math.abs(SubsystemData.AutonError.heading.toDouble()) < SubsystemData.AutonAngleStoppingDifference) {
                diffySwerve.stopDifferentialSwerve();
            } else {
                diffySwerve.setDifferentialSwerve(command, robotVelRobot, PARAMS.inPerTick * PARAMS.trackWidthTicks, voltage, feedforward);
            }


            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                diffySwerve.stopDifferentialSwerve();
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();


            txWorldTarget = OffsetTargetPositionWithVel(txWorldTarget, robotVelRobot, SubsystemData.targetPosePerpOffset);


            double txHeadingDouble = Math.atan2(txWorldTarget.heading.imag.value(), txWorldTarget.heading.real.value());
            SubsystemData.AutonError = new Pose2d(new Vector2d(txWorldTarget.position.x.value() - pose.position.x, txWorldTarget.position.y.value() - pose.position.y), txHeadingDouble - pose.heading.toDouble());

            SubsystemData.LateralPID.setVariablesTheSameAs(SubsystemData.AxialPID); // makes sure both PIDs have the same settings
            SubsystemData.AxialPID.setTarget(txWorldTarget.position.y.value());
            SubsystemData.LateralPID.setTarget(txWorldTarget.position.x.value());

            //PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>( // create my own command with proper pid values
            //new DualNum<Time>(new double[] {SubsystemData.LateralPID.getPower(pose.position.x), 1}),
            //new DualNum<Time>(new double[] {SubsystemData.AxialPID.getPower(pose.position.y), 1})),
            //new DualNum<Time>(new double[] {-1 * SubsystemData.HeadingTargetPID.getPowerWrapped(Math.toDegrees(txHeadingDouble), 360), 1}));

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>( // create my own command with proper pid values
                    new DualNum<Time>(new double[] {SubsystemData.AutonError.position.x * SubsystemData.AutonMovementGain, 1}),
                    new DualNum<Time>(new double[] {SubsystemData.AutonError.position.y * SubsystemData.AutonMovementGain, 1})),
                    new DualNum<Time>(new double[] {-1 * SubsystemData.HeadingTargetPID.getPowerWrapped(Math.toDegrees(txHeadingDouble), 360), 1}));

            // for telemetry and roadrunner's dashboard
            driveCommandWriter.write(new DriveCommandMessage(command));

            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            telemetry.addLine("Target Pose X:" + functions.round(txWorldTarget.position.x.get(0), 3) + " Y:" + functions.round(txWorldTarget.position.y.get(0), 3) + " A:" + functions.round(txHeadingDouble, 3));

            // telemetry.addData("Error Distance:", Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y));

            if (Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y) < SubsystemData.AutonStoppingDistance) {
                command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>(
                        new DualNum<Time>(new double[] {0, command.linearVel.x.get(1)}),
                        new DualNum<Time>(new double[] {0, command.linearVel.y.get(1)})),
                        command.angVel);
            }
            if (Math.abs(SubsystemData.AutonError.heading.toDouble()) < SubsystemData.AutonAngleStoppingDifference) {
                command = new PoseVelocity2dDual<Time>(new Vector2dDual<Time>(
                        command.linearVel.x,
                        command.linearVel.y),
                        new DualNum<Time>(new double[] {0, command.angVel.get(1)}));
            }

            telemetry.addLine("command:" + functions.round(t, 3) + " X:" + functions.round(command.linearVel.x.value(), 3) + " Y:" + functions.round(command.linearVel.y.value(), 3) + " A:" + functions.round(command.angVel.value(), 3));

            if (Math.hypot(SubsystemData.AutonError.position.x, SubsystemData.AutonError.position.y) < SubsystemData.AutonStoppingDistance && Math.abs(SubsystemData.AutonError.heading.toDouble()) < SubsystemData.AutonAngleStoppingDifference) {
                diffySwerve.stopDifferentialSwerve();
            } else {
                diffySwerve.setDifferentialSwerve(command, robotVelRobot, PARAMS.inPerTick * PARAMS.trackWidthTicks, voltage, feedforward);
            }


            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();

        // Roadrunner switched the x and y and made them both negative on me for some reason but heading was fine
        Twist2d RoadrunnerIsStupid = new Twist2d(new Vector2d(twist.value().line.y, twist.value().line.x), twist.value().angle);
        pose = pose.plus(RoadrunnerIsStupid);

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));
        SubsystemData.CurrentRobotPose = pose;
        PersistentDataStorage.lastRobotPose = pose;

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public void realignHeading() {
        newImu.resetYaw();
        SubsystemData.NeedToRealignHeadingHold = true;
    }


    public void updateDifferentialSwerve() {
        diffySwerve.updateKinematicDifferentialSwerve();
    }

    public void stopDifferentialSwerve() {
        diffySwerve.stopDifferentialSwerve();
        diffySwerve.updateKinematicDifferentialSwerve();
    }

    public void toggleAbsoluteDriving() {
        SubsystemData.absoluteDriving = !SubsystemData.absoluteDriving;
    }


    public void resetSwerveWheelAngles() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public Pose2dDual<Time> OffsetTargetPositionWithVel(Pose2dDual<Time> TargetPose, PoseVelocity2d VelocityPose, double Percent) {
        if (TargetPose.position.x.value() - pose.position.x == 0) {
            telemetry.addLine("Offset X: " + Percent * VelocityPose.linearVel.x + " Y: " + 0);
            return new Pose2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<>(new double[] {pose.position.x + Percent * VelocityPose.linearVel.x, TargetPose.position.x.get(1)}),
                            TargetPose.position.y),
                    TargetPose.heading);
        } else {
            double slope = (TargetPose.position.y.value() - pose.position.y) / (TargetPose.position.x.value() - pose.position.x);
            double TargetOffsetX = (VelocityPose.linearVel.x * slope * slope + pose.position.x * slope * slope - VelocityPose.linearVel.y * slope + pose.position.x) / (1 + slope * slope);
            double TargetOffsetY = (-1 * (1 / slope)) * (TargetOffsetX - pose.position.x) + pose.position.y;
            telemetry.addLine("Offset X: " +  -1 * Percent * (TargetOffsetX - pose.position.x) + " Y: " +  -1 * Percent * (TargetOffsetY - pose.position.y));
            return new Pose2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<>(new double[] {TargetPose.position.x.value() - Percent * (TargetOffsetX - pose.position.x), TargetPose.position.x.get(1)}),
                            new DualNum<>(new double[] {TargetPose.position.y.value() - Percent * (TargetOffsetY - pose.position.y), TargetPose.position.y.get(1)})),
                    TargetPose.heading);
        }
    }


    public static class RRFinishCommand implements Action { @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }}

    public static class RRPointTowardsPose implements Action {
        double targetAngle;
        ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean FirstStart = true;
        double timeout;
        public RRPointTowardsPose(Vector2d targetPose) {
            targetAngle = Math.toDegrees(Math.atan2(targetPose.y, targetPose.x));
            timeout = 2000 * functions.angleDifference(SubsystemData.CurrentRobotPose.heading.toDouble(), targetAngle, 360) / 360;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (FirstStart) {
                timeoutTimer.reset();
                FirstStart = false;
            }

            if (Math.abs(functions.angleDifference(SubsystemData.CurrentRobotPose.heading.toDouble(), targetAngle, 360)) < SubsystemData.AutonAngleStoppingDifference || timeoutTimer.time() > timeout) {
                diffySwerve.stopDifferentialSwerve();
                return false;
            } else {
                double turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(targetAngle, 360);
                diffySwerve.driveDifferentialSwerve(0, 0, turn, 0);
                return true;
            }
        }
    }

    public Action PointTowardsPose(Vector2d targetPose) {
        return new RRPointTowardsPose(targetPose);
    }


    public static class RRPointTowardsAngle implements Action {
        double targetAngle;
        ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean FirstStart = true;
        double timeout;
        public RRPointTowardsAngle(double targetAngle) {
            this.targetAngle = targetAngle;
            timeout = 2000 * functions.angleDifference(SubsystemData.CurrentRobotPose.heading.toDouble(), targetAngle, 360) / 360;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (FirstStart) {
                timeoutTimer.reset();
                FirstStart = false;
            }

            if (Math.abs(functions.angleDifference(SubsystemData.CurrentRobotPose.heading.toDouble(), targetAngle, 360)) < SubsystemData.AutonAngleStoppingDifference || timeoutTimer.time() > timeout) {
                diffySwerve.stopDifferentialSwerve();
                return false;
            } else {
                double turn = -1 * SubsystemData.HeadingTargetPID.getPowerWrapped(targetAngle, 360);
                diffySwerve.driveDifferentialSwerve(0, 0, turn, 0);
                return true;
            }
        }
    }


    public Action PointTowardsAngle(double targetAngleDegrees) {
        return new RRPointTowardsAngle(targetAngleDegrees);
    }


}
