
package frc.robot;

import com.MAutils.RobotControl.DeafultRobot;

import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Util.ActiveUtil;

public class Robot extends DeafultRobot {

  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;

  public Robot() {
    super();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    ActiveUtil.startTeleop();

    if (SuperStructure.isRobotInAir()) {
      Climb.getInstance().setState(ClimbConstnats.DOWN);
    }
  }


  @Override
  public void teleopPeriodic() {
    super.teleopPeriodic();
    ActiveUtil.checkShift();
  }

  
}
