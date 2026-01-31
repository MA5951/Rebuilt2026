
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.AprilTagCamera;
import com.MAutils.Vision.IOs.LimelightIO;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Swerve.Swerve;

public class VisionConstants {


    public static final Translation2d FRONTLL_OFFSET = new Translation2d(0.0, 0.0);

    public static final FiltersConfig DEFAULT_FILTERS_CONFIG = new FiltersConfig();

    public static final AprilTagCamera FRONT_LL = new AprilTagCamera(
            new LimelightIO("FrontLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
            Swerve.getInstance().getGyroYawSupplier());

    public static final AprilTagCamera BACK_LL = new AprilTagCamera(
            new LimelightIO("BackLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
            Swerve.getInstance().getGyroYawSupplier());

}
