package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = robot.imu.getRobotYawPitchRollAngles();
        double heading = robotOrientation.getYaw(AngleUnit.DEGREES);

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();


        boolean AbsoluteSetting = false;
        //double AbsDrivingDirection = OpVariableStorage.rotationChange; // allows information to travel from auton to teleop
        double FrameRate = 0;
        double lastAngle = 0;
        double throttleControl = 0.5;
        boolean reset_heading = false;
        boolean absoluteDriving = true;
        double driveDirection = 0;
        double joystickMagnitude = 0;
        double drivePower = 0;

        // temporary TurnControl PID
        double PIDVar = 0; // 0 = kp, 1 = ki, 2 = kd
        double PIDChangeIncrement = 0.01;


        // RoadRunner stuff
        //List<Integer> lastTrackingEncPositions = new ArrayList<>();
        //List<Integer> lastTrackingEncVels = new ArrayList<>();
        //StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        //Starting Position
        //myLocalizer.setPoseEstimate(OpVariableStorage.currentPose);

        //telemetry.addData("StartingPose:", OpVariableStorage.currentPose);
        //telemetry.update();


        waitForStart();

        // IMPORTANT: Start configuration has differential swerve wheels inner bevel gear facing the left side of the robot (wheels point forward)

        while (opModeIsActive() && !isStopRequested()) {
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            heading = robotOrientation.getYaw(AngleUnit.DEGREES);

            // Controller inputs
            driveDirection = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
            joystickMagnitude = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            drivePower = Math.abs(joystickMagnitude) * joystickMagnitude; // Makes it easier to control the robot
            if (gamepad1.right_trigger > 0) {
                throttleControl = 0.5 + gamepad1.right_trigger / 2;
            } else throttleControl = 0.5;


            if (absoluteDriving) {
                driveDirection = robot.angleDifference(driveDirection + heading, 0, 360);
            }

            robot.driveDiffySwerveWithControllers(driveDirection, drivePower, -0.6 * Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x, throttleControl);


            if (gamepad1.y && !reset_heading) {
                robot.imu.resetYaw();
                reset_heading = true;
            } else if (gamepad1.x && !reset_heading) {
                absoluteDriving = !absoluteDriving;
            } else reset_heading  = false;


            // Allows changing PID variables whilst on the field during testing:
            if (gamepad1.left_bumper) PIDChangeIncrement = 0.01;
            else PIDChangeIncrement = 0.0001;

            if (gamepad1.dpad_right && !AbsoluteSetting) {
                PIDVar = PIDVar + 1;
                if (PIDVar > 2) PIDVar = 0;
                AbsoluteSetting = true;
            } else if (gamepad1.dpad_left && !AbsoluteSetting) {
                PIDVar = PIDVar - 1;
                if (PIDVar < 0) PIDVar = 2;
                AbsoluteSetting = true;

            } else if (gamepad1.dpad_up && !AbsoluteSetting) {
                if (PIDVar == 0) robot.PosKp = (robot.PosKp + PIDChangeIncrement);
                else if (PIDVar == 1) robot.PosKi = (robot.PosKi + PIDChangeIncrement);
                else if (PIDVar == 2) robot.PosKd = (robot.PosKd + PIDChangeIncrement);
                AbsoluteSetting = true;
            } if (gamepad1.dpad_down) {
                if (PIDVar == 0 && robot.PosKp > 0) robot.PosKp = (robot.PosKp - PIDChangeIncrement);
                else if (PIDVar == 1 && robot.PosKi > 0) robot.PosKi = (robot.PosKi - PIDChangeIncrement);
                else if (PIDVar == 2 && robot.PosKd > 0) robot.PosKd = (robot.PosKd - PIDChangeIncrement);
                AbsoluteSetting = true;

            } else if (!gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_down) AbsoluteSetting = false;


            // Arm extension
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // extend but not past max and min

            } else {
                // hold position

            }




            FrameRate = Math.round((1 / (mRuntime.time() - LastTime)) * 1000);

            telemetry.addData("FPS:", FrameRate);
            telemetry.addData("MSPerFrame:", (mRuntime.time() - LastTime));
            LastTime = mRuntime.time();
            telemetry.addData("Heading:", heading);
            if (!absoluteDriving) telemetry.addLine("Absolute Driving Disabled");
            //telemetry.addData("X:", myPose.getX()); // Roadrunner implementation (for odometry)
            //telemetry.addData("Y:", myPose.getY());
            //telemetry.addData("Heading:", Math.toDegrees(myPose.getHeading()));
            telemetry.addData("Gyro Yaw:", robotOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Gyro Pitch:", robotOrientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Gyro Roll:", robotOrientation.getRoll(AngleUnit.DEGREES));
            telemetry.addLine(" ");
            if (PIDVar == 0) telemetry.addLine("Setting: Kp");
            else if (PIDVar == 1) telemetry.addLine("Setting: Ki");
            else if (PIDVar == 2) telemetry.addLine("Setting: Kd");
            telemetry.addData("Kp: ", robot.PosKp);
            telemetry.addData("Ki: ", robot.PosKi);
            telemetry.addData("Kd: ", robot.PosKd);
            telemetry.addData("Direction:", driveDirection);
            telemetry.addData("Magnitude:", joystickMagnitude);
            telemetry.addData("Right:", robot.getCurrentRightDiffAngle());
            telemetry.addData("Left:", robot.getCurrentLeftDiffAngle());
            telemetry.update();
        }
    }
}
