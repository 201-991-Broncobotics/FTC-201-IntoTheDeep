package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SubsystemData;

public class IMUThread extends Thread {

    private final ElapsedTime ThreadTimer, imuNotWorkingTimer;

    private final IMU imu;

    @Override
    public void run() {
        while (SubsystemData.IMUThreadActive) {
            ThreadTimer.reset();
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            SubsystemData.IMUAngles = robotOrientation;
            SubsystemData.IMUAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Check if Imu had an ESD event and murdered itself
            if (robotOrientation.getYaw(AngleUnit.DEGREES) == 0 && robotOrientation.getPitch(AngleUnit.DEGREES) == 0 && robotOrientation.getRoll(AngleUnit.DEGREES) == 0) {
                if (imuNotWorkingTimer.time() > 3000) {
                    SubsystemData.IMUWorking = false;
                }
            } else imuNotWorkingTimer.reset();

            SubsystemData.IMUThreadTime = ThreadTimer.time();
        }
    }

    public IMUThread(IMU inputIMU) {
        imu = inputIMU;
        SubsystemData.IMUAngles = imu.getRobotYawPitchRollAngles();
        SubsystemData.IMUAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        ThreadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuNotWorkingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

}
