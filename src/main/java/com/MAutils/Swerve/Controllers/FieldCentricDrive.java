
package com.MAutils.Swerve.Controllers;

import java.util.function.Supplier;

import com.MAutils.Controllers.MAController;
import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.SwerveController;
import com.MAutils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class FieldCentricDrive extends SwerveController {

    private MAController controller;
    private final SwerveSystemConstants constants;

    private double xyScaler = 1;
    private double omegaScaler = 1;

    private Supplier<GyroData> gyroDataSupplier;
    private double angleOffset = 0;

    public FieldCentricDrive(MAController controller, SwerveSystemConstants constants, Supplier<GyroData> gyroDataSupplier) {
        super("Field Centric Drive");
        this.constants = constants;
        this.controller = controller;
        this.gyroDataSupplier = gyroDataSupplier;
    }

    public FieldCentricDrive withSclers(double xyScaler, double omegaScaler) {
        this.omegaScaler = omegaScaler;
        this.xyScaler = xyScaler;
        return this;
    }

    public void updateOffset() {
        angleOffset = gyroDataSupplier.get().yaw;
    }

    public void setOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }

    public double  getOffset() {
        return angleOffset ;
    }

    public void updateSpeeds() {
        speeds.vxMetersPerSecond = -controller.getLeftY(true, xyScaler) * constants.MAX_VELOCITY;
        speeds.vyMetersPerSecond = -controller.getLeftX(true, xyScaler) * constants.MAX_VELOCITY;
        speeds.omegaRadiansPerSecond = -controller.getRightX(true, omegaScaler) * constants.MAX_ANGULAR_VELOCITY;


        speeds = ChassisSpeedsUtil.FromFieldToRobot(speeds,
                Rotation2d.fromDegrees(gyroDataSupplier.get().yaw - angleOffset));

    }

    @Override
    public void logController() {
        super.logController();
        MALog.log("Subsystems/Swerve/Controllers/Field Centric Drive/Angle Offset", angleOffset);
    }

}
