
package com.MAutils.Components;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstatns;


/*
 * Defines different camera types and their properties.
 */
public class CameraTypes {

    public enum Cameras {
        LL4(1280, 800, 82, 50),
        LL3G(1280, 800, 86, 35),
        LL3(1280, 960, 70, 20),
        LL2_PLUS(640, 480, 50, 11);

        public final int width;
        public final int height;
        public final int fov;
        public final int simFps;

        Cameras(int width, int height, int fov, int simFps) {
            this.width = width;
            this.height = height;
            this.fov = fov;
            this.simFps = simFps;
        }

        public SimCameraProperties getSimulationProp() {
            return new SimCameraProperties().setCalibration(width, height, Rotation2d.fromDegrees(fov))
                    .setCalibError(RobotConstatns.CameraTypesConstatnt.AVG_ERROR_PX, RobotConstatns.CameraTypesConstatnt.ERROR_STD_DEV_PX) 
                    .setFPS(simFps)
                    .setAvgLatencyMs(RobotConstatns.CameraTypesConstatnt.AVG_LATENCY_MS) 
                    .setLatencyStdDevMs(RobotConstatns.CameraTypesConstatnt.LATENCY_STD_DEV_MS);
        }
    }

}
