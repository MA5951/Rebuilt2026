
package frc.robot;

import com.MAutils.RobotControl.DeafultRobot;

import frc.robot.Util.ActiveUtil;

public class Robot extends DeafultRobot {

  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;

  public Robot() {
    super();
    m_robotContainer = new RobotContainer();
  }

  //TODO what about the auto? we need to talk about it

  @Override
  public void teleopInit() {
    super.teleopInit();
    ActiveUtil.startTeleop();
  }

  @Override
  public void teleopPeriodic() {
    super.teleopPeriodic();
    ActiveUtil.checkShift();
  }

  
}
