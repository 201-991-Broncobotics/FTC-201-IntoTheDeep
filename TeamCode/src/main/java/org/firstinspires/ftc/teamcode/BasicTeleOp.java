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


        boolean ButtonPressed = false;
        boolean PIDIncrementButtonPressed = false;
        //double AbsDrivingDirection = OpVariableStorage.rotationChange; // allows information to travel from auton to teleop
        double FrameRate = 0;
        double throttleControl = 0.5;
        boolean reset_heading = false;
        boolean absoluteDriving = true;
        double driveDirection = 0;
        double joystickMagnitude = 0;
        double drivePower = 0;
        double extensionTarget = 0;
        double extensionSpeed = 0;
        double extensionPower = 0;
        double pivotTarget = 0;
        double pivotSpeed = 0;
        double pivotPower = 0;
        double clawY = 0; // eventually the pivot, extension, and drivetrain should work together to move the claw to a specific x,y point
        double clawX = 0;
        double PIDEditingButtonPressedTime = 0; // in ms

        // temporary TurnControl PID
        double PIDVar = 0;
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


            if (gamepad1.y && !reset_heading) { // reset absolute driving
                robot.imu.resetYaw();
                reset_heading = true;
            } else if (gamepad1.x && !reset_heading) { // toggle absolute driving
                absoluteDriving = !absoluteDriving;
            } else reset_heading  = false;


            // Allows changing PID variables whilst on the field during testing:
            if (gamepad1.left_bumper) PIDChangeIncrement = 0.01;
            else PIDChangeIncrement = 0.0001;

            if (gamepad1.dpad_right && !ButtonPressed) { // cycle through which PID variable is going to be edited
                PIDVar = PIDVar + 1;
                if (PIDVar > 9) PIDVar = 0;
                ButtonPressed = true;
            } else if (gamepad1.dpad_left && !ButtonPressed) {
                PIDVar = PIDVar - 1;
                if (PIDVar < 0) PIDVar = 9;
                ButtonPressed = true;
            } else if (!gamepad1.dpad_right && !gamepad1.dpad_left) ButtonPressed = false;

            // if the button is held for more than a second, add or subtract constantly
            if ((gamepad1.dpad_up || gamepad1.dpad_down) && !(PIDIncrementButtonPressed && (mRuntime.time() - PIDEditingButtonPressedTime) < 1000)) {
                if (gamepad1.dpad_down)
                    PIDChangeIncrement = -PIDChangeIncrement; // subtract if down
                if (PIDVar == 1) robot.PosKp = (robot.PosKp + PIDChangeIncrement); // Diffy
                else if (PIDVar == 2) robot.PosKi = (robot.PosKi + PIDChangeIncrement);
                else if (PIDVar == 3) robot.PosKd = (robot.PosKd + PIDChangeIncrement);
                else if (PIDVar == 4) robot.PivotKp = (robot.PivotKp + PIDChangeIncrement); // Pivot
                else if (PIDVar == 5) robot.PivotKi = (robot.PivotKi + PIDChangeIncrement);
                else if (PIDVar == 6) robot.PivotKd = (robot.PivotKd + PIDChangeIncrement);
                else if (PIDVar == 7) robot.ExtendKp = (robot.ExtendKp + PIDChangeIncrement); // Extension
                else if (PIDVar == 8) robot.ExtendKi = (robot.ExtendKi + PIDChangeIncrement);
                else if (PIDVar == 9) robot.ExtendKd = (robot.ExtendKd + PIDChangeIncrement);
                if (!PIDIncrementButtonPressed) { // only happens once when the button is first pressed
                    PIDEditingButtonPressedTime = mRuntime.time(); // log the time that the button started being pressed
                    PIDIncrementButtonPressed = true;
                }
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                PIDIncrementButtonPressed = false;
            }


            // Arm extension
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // extend but not past max and min
                extensionSpeed = Math.abs(gamepad2.left_stick_y) * gamepad2.left_stick_y * 30; // mm per second
                if (!(FrameRate == 0)) extensionTarget = extensionTarget + extensionSpeed / FrameRate;  // Change target by set speed as long as the frame rate isn't 0
                if (extensionTarget > 696) extensionTarget = 696; // if target is beyond max, reset to max
                else if (extensionTarget < 0) extensionTarget = 0;
            } else if (gamepad2.dpad_right) extensionTarget = 696 - 50; // preset extensions to travel to at max speed
            else if (gamepad2.dpad_left) extensionTarget = 0;

            extensionPower = robot.ExtensionPID(extensionTarget, robot.LinearSlideLength()); // go to and hold at target
            //robot.Extension.setPower(extensionPower);


            // Arm pivot
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                // extend but not past max and min
                pivotSpeed = Math.abs(gamepad2.right_stick_y) * gamepad2.right_stick_y * 20; // degrees per second
                if (!(FrameRate == 0)) pivotTarget = pivotTarget + pivotSpeed / FrameRate; // Change target by set speed as long as the frame rate isn't 0
                if (pivotTarget > 90) pivotTarget = 90; // if target is beyond max, reset to max
                else if (pivotTarget < 0) pivotTarget = 0;
            } else if (gamepad2.dpad_up) pivotTarget = 90; // preset angles to travel to at max speed
            else if (gamepad2.dpad_down) pivotTarget = 0;
            pivotPower = robot.PivotPID(pivotTarget, robot.PivotAngle());

            /* //i don't want to break the bottom plate again
            if (Math.abs(pivotPower) > 0.2) { // go to and hold at target but keep max power under 0.2
                robot.Pivot.setPower(Math.signum(pivotPower) * 0.2);
            } else robot.Pivot.setPower(pivotPower);
            */


            FrameRate = (1 / (mRuntime.time() - LastTime)) * 1000;

            telemetry.addData("FPS:", Math.round(FrameRate));
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
            if (PIDVar == 0) telemetry.addLine("Not Editing PIDs");
            else if (PIDVar == 1) telemetry.addData("Editing: Diffy Kp - ", robot.PosKp);
            else if (PIDVar == 2) telemetry.addData("Editing: Diffy Ki - ", robot.PosKi);
            else if (PIDVar == 3) telemetry.addData("Editing: Diffy Kd - ", robot.PosKd);
            else if (PIDVar == 4) telemetry.addData("Editing: Pivot Kp - ", robot.PivotKp);
            else if (PIDVar == 5) telemetry.addData("Editing: Pivot Ki - ", robot.PivotKi);
            else if (PIDVar == 6) telemetry.addData("Editing: Pivot Kd - ", robot.PivotKd);
            else if (PIDVar == 7) telemetry.addData("Editing: Extension Kp - ", robot.ExtendKp);
            else if (PIDVar == 8) telemetry.addData("Editing: Extension Ki - ", robot.ExtendKi);
            else if (PIDVar == 9) telemetry.addData("Editing: Extension Kd - ", robot.ExtendKd);
            telemetry.addLine(" ");
            telemetry.addData("Extension Length:", robot.LinearSlideLength());
            telemetry.addData("Extension Target:", extensionTarget);
            telemetry.addData("Extension Power:", extensionPower);
            telemetry.addData("Pivot Angle:", robot.PivotAngle());
            telemetry.addData("Pivot Target:", pivotTarget);
            telemetry.addData("Pivot Power:", pivotPower);
            telemetry.update();
        }
    }
}
