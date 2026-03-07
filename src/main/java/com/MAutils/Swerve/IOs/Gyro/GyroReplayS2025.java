
package com.MAutils.Swerve.IOs.Gyro;


import com.MAutils.Logger.MALog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;



public class GyroReplayS2025 implements GyroIO {


    public GyroReplayS2025() {
    }

    public void resetYaw(double yaw) {
    }

    public void updateGyroData(GyroData gyroData) {
        gyroData.isConnected = true;
        gyroData.yaw = NetworkTableInstance.getDefault().getEntry("/MALog//Subsystems/Swerve/Piegon/Yaw").getDouble(0);
        gyroData.yawVelocity = 0;
        gyroData.pitch = NetworkTableInstance.getDefault().getEntry("/MALog//Subsystems/Swerve/Piegon/Pitch").getDouble(0);
        gyroData.roll = NetworkTableInstance.getDefault().getEntry("/MALog//Subsystems/Swerve/Piegon/Roll").getDouble(0);
        gyroData.accelX = NetworkTableInstance.getDefault().getEntry("/MALog//Subsystems/Swerve/Piegon/Accel X").getDouble(0);
        gyroData.accelY = NetworkTableInstance.getDefault().getEntry("/MALog//Subsystems/Swerve/Piegon/Accel Y").getDouble(0);

        gyroData.odometryYawTimestamps = new double[] {
            Timer.getFPGATimestamp()
        };
        gyroData.odometryYawPositions = new Rotation2d[] {
            Rotation2d.fromDegrees(gyroData.yaw)
        };
    }

}
