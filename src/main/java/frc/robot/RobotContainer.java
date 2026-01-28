
package frc.robot;

import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.RobotControl.StateTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class RobotContainer extends DeafultRobotContainer {

  public RobotContainer() {
    super();
  }

  @Override
  public void configAuto() {
    throw new UnsupportedOperationException("Unimplemented method 'configAuto'");
  }

  @Override
  public void configBinding() {
    T(StateTrigger.T(() -> getDriverController().getMiddle(), RobotConstants.IDLE));

    T(StateTrigger.T(
        () -> ((getRobotState() == RobotConstants.INTAKE_DEPLOY || getRobotState() == RobotConstants.INTAKE_ROLLER)
            && (!getDriverController().getR1()) || SuperStructure.isFull()) ||
            (getRobotState() == RobotConstants.EJECT && !getDriverController().getActionsRight())
            || (getRobotState() == RobotConstants.FEEDING && (!getDriverController().getActionsRight() || !SuperStructure.isBalls()))
            ||(getRobotState() == RobotConstants.FEEDING_IN_MOTION && !getDriverController().getR2())
            || (getRobotState() == RobotConstants.SHOOTING && (!getDriverController().getR2()|| !SuperStructure.isBalls() 
            || (!SuperStructure.isInActive() &&  SuperStructure.getTimePastActive() < 1.5)))
            || getRobotState() == RobotConstants.SHOOTING_PRESETS && !getDriverController().getR2()
            , RobotConstants.IDLE_INTAKE));

    T(StateTrigger.T(() -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_DEPLOY));

    T(StateTrigger.T(() -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() == SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_ROLLER));

    T(StateTrigger.T(() -> getDriverController().getL1() && SuperStructure.isInActive() && SuperStructure.getTimeUntilActive() > 2 && getRobotState() == RobotConstants.UNSTUCK,
        RobotConstants.INTAKE_ROLLER));
    

    // Internal shooter stats

    new Trigger(() -> SuperStructure.isBalls() && SuperStructure.isInTheAlinceZone() 
    && getRobotState() != RobotConstants.SHOOTING && getRobotState() != RobotConstants.SHOOTING_PRESETS 
    && getRobotState() != RobotConstants.FEEDING && getRobotState() != RobotConstants.FEEDING_IN_MOTION 
    && getRobotState() != RobotConstants.EJECT 
    && ((!SuperStructure.isInActive()  && SuperStructure.getTimeUntilActive() < 5) || SuperStructure.isInActive()) )
    .onTrue( new InstantCommand(() -> Shooter.getInstance().setState(ShooterConstants.WARMUP)));// change to variable

   T(StateTrigger.T(() -> !SuperStructure.isBalls() || !SuperStructure.isInTheAlinceZone() || SuperStructure.getTimePastActive() > 1 // change to variable
   , RobotConstants.IDLE_SHOOTER));

   

  }

}
