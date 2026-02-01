
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.AprilTagCamera;
import com.MAutils.Vision.IOs.LimelightIO;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Swerve.Swerve;

public class VisionConstants {


    public static final Translation2d FRONTLL_OFFSET = new Translation2d(0.0, 0.0);

    public static final FiltersConfig DEFAULT_FILTERS_CONFIG = new FiltersConfig(); //TODO make sure that all the filters in the mautil CR are on velcoity, jump, angle etc
    //TODO plue the filter between  the two cam, and also the on ramp one
    //TODO add to the configh that you cant reast into the hub and other filde elements

    //TODO you need to filter when shhoting that the fron can only see the main tag, so its must have a set filter func 

    public static final AprilTagCamera FRONT_LL = new AprilTagCamera(
            new LimelightIO("FrontLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
            Swerve.getInstance().getGyroYawSupplier());

    public static final AprilTagCamera BACK_LL = new AprilTagCamera(
            new LimelightIO("BackLL", Swerve.getInstance().getGyroYawSupplier()), DEFAULT_FILTERS_CONFIG,
            Swerve.getInstance().getGyroYawSupplier());

}
