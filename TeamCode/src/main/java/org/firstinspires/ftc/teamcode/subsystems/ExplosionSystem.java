package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.SubsystemData;

public class ExplosionSystem extends SubsystemBase {

    Telemetry telemetry;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public ExplosionSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotorEx.class, "R3");
        leftBack = hardwareMap.get(DcMotorEx.class, "R4");
        rightBack = hardwareMap.get(DcMotorEx.class, "R1");
        rightFront = hardwareMap.get(DcMotorEx.class, "R2");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void runControls() {
        double throttle = SubsystemData.driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (SubsystemData.driver.getButton(GamepadKeys.Button.Y)) {
            leftFront.setPower(throttle * Settings.maxDrivetrainMotorPower);
            leftBack.setPower(throttle * Settings.maxDrivetrainMotorPower);
            rightBack.setPower(throttle * Settings.maxDrivetrainMotorPower);
            rightFront.setPower(throttle * Settings.maxDrivetrainMotorPower);
        } else if (SubsystemData.driver.getButton(GamepadKeys.Button.A)) {
            leftFront.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            leftBack.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            rightBack.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            rightFront.setPower(-throttle * Settings.maxDrivetrainMotorPower);
        } else if (SubsystemData.driver.getButton(GamepadKeys.Button.X)) {
            leftFront.setPower(throttle * Settings.maxDrivetrainMotorPower);
            leftBack.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            rightBack.setPower(throttle * Settings.maxDrivetrainMotorPower);
            rightFront.setPower(-throttle * Settings.maxDrivetrainMotorPower);
        } else if (SubsystemData.driver.getButton(GamepadKeys.Button.B)) {
            leftFront.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            leftBack.setPower(throttle * Settings.maxDrivetrainMotorPower);
            rightBack.setPower(-throttle * Settings.maxDrivetrainMotorPower);
            rightFront.setPower(throttle * Settings.maxDrivetrainMotorPower);
        } else {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }

        telemetry.addData("Current Applied Motor Power", throttle * Settings.maxDrivetrainMotorPower);
        telemetry.addData("LeftFront Current", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("leftBack Current", leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightBack Current", rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightFront Current", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

    }


}
