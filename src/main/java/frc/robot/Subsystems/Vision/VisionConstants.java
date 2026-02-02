
package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;
import java.util.zip.GZIPInputStream;

import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.AprilTagCamera;
import com.MAutils.Vision.IOs.LimelightIO;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Swerve.Swerve;

public class VisionConstants {

        public static final Translation2d FRONTLL_OFFSET = new Translation2d(0.0, 0.0);
        public static final double FRONTLL_ANGLE = 0.0;
        public static final double FRONT_LL_HIGHT = 0;


        public static final FiltersConfig DEFAULT_FILTERS_CONFIG = new FiltersConfig();

        public static final AprilTagCamera FRONT_LL = new AprilTagCamera(
                        new LimelightIO("FrontLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
                        Swerve.getInstance().getGyroYawSupplier());
                        //TODO lets go over the filters togher

        public static final AprilTagCamera BACK_LL = new AprilTagCamera(
                        new LimelightIO("BackLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
                        Swerve.getInstance().getGyroYawSupplier());

}
