
package frc.robot.Commands;

import com.MAutils.Swerve.SwerveSystemController;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;

public class SwerveController extends SwerveSystemController {
    public static int isAbs = 0;

    public SwerveController() {
        super(Swerve.getInstance(), SwerveConstants.SWERVE_CONSTANTS, RobotContainer.getDriverController());
    }

    public void ConfigControllers() {
    }

    public void SetSwerveState() {

        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING ||  
        (RobotContainer.getRobotState() == RobotConstants.UNSTUCK && RobotContainer.getLastRobotState() == RobotConstants.SHOOTING)) {
            if (SuperStructure.isMainTag() && isAbs < 50) {
                setState(SwerveConstants.SHOOTING_REL);
            } else {
                isAbs ++;
                setState(SwerveConstants.SHOOTING_ABS);
            }
        } 
        else if (RobotContainer.getRobotState() == RobotConstants.FEEDING || 
        (RobotContainer.getRobotState() == RobotConstants.UNSTUCK && RobotContainer.getLastRobotState() == RobotConstants.FEEDING)) {
            setState(SwerveConstants.FEEDING);
        } 
        else if (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION || 
        (RobotContainer.getRobotState() == RobotConstants.UNSTUCK && RobotContainer.getLastRobotState() == RobotConstants.FEEDING_IN_MOTION)) {
            setState(SwerveConstants.FEEDING_IN_MOTION);
        } 
        else {
            if (RobotContainer.getDriverController().getL2()) {
                setState(SwerveConstants.FIELD_CENTRIC_40);
            } else {
                setState(SwerveConstants.FIELD_CENTRIC);
            }
        }

   

    }

    

}
