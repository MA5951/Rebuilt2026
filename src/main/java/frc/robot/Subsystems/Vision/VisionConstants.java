
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.AprilTagCamera;
import com.MAutils.Vision.IOs.LimelightIO;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Swerve.Swerve;

public class VisionConstants {

        public static final Translation2d FRONTLL_OFFSET = new Translation2d(0.327, 0.186);
        public static final double FRONT_LL_HIGHT = 0.408;

        public static final FiltersConfig DEFAULT_FILTERS_CONFIG = new FiltersConfig();

        public static final AprilTagCamera FRONT_LL = new AprilTagCamera(
                        new LimelightIO("limelight-frontll", Swerve.getInstance().getAbsYawSupplier()),
                        DEFAULT_FILTERS_CONFIG,
                        Swerve.getInstance().getAbsYawSupplier(), () -> Swerve.getInstance().getGyroData().yawVelocity,
                        () -> new ChassisSpeeds());//Swerve.getInstance().getChassisSpeeds()

        public static final AprilTagCamera BACK_LL = new AprilTagCamera(
                        new LimelightIO("limelight-backll--", Swerve.getInstance().getAbsYawSupplier()),
                        DEFAULT_FILTERS_CONFIG,
                        Swerve.getInstance().getAbsYawSupplier(), () -> Swerve.getInstance().getGyroData().yawVelocity,
                        () -> Swerve.getInstance().getChassisSpeeds());

}
