
package frc.robot.Commands;

import com.MAutils.Swerve.SwerveSystemController;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;

public class SwerveAutoController extends SwerveSystemController {
    public static int isAbs = 0;

    public SwerveAutoController() {
        super(Swerve.getInstance(), SwerveConstants.SWERVE_CONSTANTS, RobotContainer.getDriverController());
    }

    public void ConfigControllers() {
    }

    public void SetSwerveState() {

         if (RobotContainer.getRobotState() == RobotConstants.SHOOTING ||
                    (RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                            && RobotContainer.getLastRobotState() == RobotConstants.SHOOTING)) {

                 if (SuperStructure.isMainTag() && isAbs < 3) {
                     setState(SwerveConstants.SHOOTING_REL);
                     if (isAbs == 1) {
                         isAbs = 2;
                     }
                 } else {
                    setState(SwerveConstants.SHOOTING_ABS);
                     if (isAbs == 2 || isAbs == 3) {
                         isAbs = 3;
                     } else {
                     isAbs = 1;
                     }
                 }
                 
                
            } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING ||
                    (RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                            && RobotContainer.getLastRobotState() == RobotConstants.FEEDING)) {
                setState(SwerveConstants.FEEDING);
            } else {
                if (RobotContainer.getDriverController().getL2()) {
                    setState(SwerveConstants.FIELD_CENTRIC_40);
                } else {
                    setState(SwerveConstants.FIELD_CENTRIC);
                }
            // }
        }



    }

}
