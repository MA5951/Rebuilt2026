
package frc.robot;

import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.RobotControl.StateTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Util.ActiveUtil;

public class RobotContainer extends DeafultRobotContainer {

  private static final double TIME_UNTIL_ACTIVE = 1;
  private static final double TIME_PAST_ACTIVE = 1;


  public RobotContainer() {
    super();
  }

  @Override
  public void configAuto() {
    throw new UnsupportedOperationException("Unimplemented method 'configAuto'");
  }

  @Override
  public void configBinding() {
    new Trigger(() -> getDriverController().getActionsUp()).onTrue(
        new InstantCommand(() -> SwerveConstants.FIELD_CENTRIC_DRIVE.updateOffset()));

    new Trigger(() -> getDriverController().getOptionsRight()).onTrue(
        new InstantCommand(() -> SuperStructure.setAutomatic(!SuperStructure.isAutomatic())));

    T(StateTrigger.T(() -> getDriverController().getMiddle(), RobotConstants.IDLE));

    T(StateTrigger.T(
        () -> ((getRobotState() == RobotConstants.INTAKE_DEPLOY || getRobotState() == RobotConstants.INTAKE_ROLLER)
            && (!getDriverController().getR1()) || SuperStructure.isFull()) ||
            (getRobotState() == RobotConstants.EJECT && !getDriverController().getActionsRight())
            || (getRobotState() == RobotConstants.FEEDING && (!getDriverController().getActionsLeft() || !SuperStructure.isBalls()))
            ||(getRobotState() == RobotConstants.FEEDING_IN_MOTION && !getDriverController().getR2())
            || (getRobotState() == RobotConstants.SHOOTING && (!getDriverController().getL1()|| !SuperStructure.isBalls() 
            || (!ActiveUtil.isActive() &&  ActiveUtil.getTimePastActive() < 1.5)))
            || getRobotState() == RobotConstants.SHOOTING_PRESETS && !getDriverController().getR2()
            , RobotConstants.IDLE_INTAKE)); //TODO if you dont in actic you whant the shooter to go idle 

    T(StateTrigger.T(() -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_DEPLOY)); 

    T(StateTrigger.T(() -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() == SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_ROLLER)); //TODO go to idle

    T(StateTrigger.T(() -> getDriverController().getL1() && (ActiveUtil.isActive() || ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE || ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE)  && 
    getRobotState() == RobotConstants.UNSTUCK && !SuperStructure.isAutomatic(),
        RobotConstants.SHOOTING)); //TODO need to add ! to the unstuck and why you need the ! in SuperStructure.isAutomatic()
        //TODO need to add that you can shoot only in a valid point

    T(StateTrigger.T(() -> getDriverController().getActionsRight() && getRobotState() != RobotConstants.UNSTUCK,
        RobotConstants.EJECT ));

    T(StateTrigger.T(()-> getDriverController().getActionsLeft() && getRobotState() != RobotConstants.UNSTUCK, 
        RobotConstants.FEEDING));

    T(StateTrigger.T(() -> getDriverController().getR2() && getRobotState() != RobotConstants.UNSTUCK, 
        RobotConstants.FEEDING_IN_MOTION));

    T(StateTrigger.T(() -> getDriverController().getL2() && SuperStructure.isAutomatic() && getRobotState() != RobotConstants.UNSTUCK, 
        RobotConstants.SHOOTING_PRESETS)); //TODO her you need to add the ! to the !SuperStructure.isAutomatic(),

    T(StateTrigger.T(() -> (getRobotState() == RobotConstants.SHOOTING || getRobotState() == RobotConstants.SHOOTING_PRESETS
        || getRobotState() == RobotConstants.EJECT || getRobotState() == RobotConstants.FEEDING 
        || getRobotState() == RobotConstants.FEEDING_IN_MOTION) && SuperStructure.isTransferStuck(),
        RobotConstants.UNSTUCK));

    T(StateTrigger.T(() -> getRobotState() == RobotConstants.UNSTUCK && !SuperStructure.isTransferStuck() , getLastRobotState()));

    T(StateTrigger.T(() -> getDriverController().getActionsLeft() 
    && Math.abs(Climb.getInstance().getPosition() - ClimbConstnats.CLOSE_POSITION) < ClimbConstnats.TOLERANCE_FOR_TRIGGER 
    && SuperStructure.getTimeLeft() < 30, RobotConstants.PRECLIMB)); //TODO add that its not in tha air or current stat ist climb

    T(StateTrigger.T(() -> getDriverController().getActionsLeft() 
    && Math.abs(Climb.getInstance().getPosition() - ClimbConstnats.OPEN_POSITION) < ClimbConstnats.TOLERANCE_FOR_TRIGGER 
    && SuperStructure.getTimeLeft() < 30, RobotConstants.CLIMB));

    // Internal climb stats
    new Trigger(() -> getDriverController().getActionsLeft() 
    && SuperStructure.getTimeLeft() < 30 && SuperStructure.isRobotInAir() 
    && (Math.abs(Climb.getInstance().getPosition() - ClimbConstnats.CLOSE_POSITION) < ClimbConstnats.TOLERANCE_FOR_TRIGGER
     || SuperStructure.getTimeLeft() > 20)) //TODO and check if we are in the air
    .onTrue( new InstantCommand(() -> Climb.getInstance().setState(ClimbConstnats.DOWN)));

    // Internal shooter stats

    new Trigger(() -> SuperStructure.isBalls() && SuperStructure.isInTheAlinceZone() 
    && getRobotState() != RobotConstants.SHOOTING && getRobotState() != RobotConstants.SHOOTING_PRESETS 
    && getRobotState() != RobotConstants.FEEDING && getRobotState() != RobotConstants.FEEDING_IN_MOTION 
    && getRobotState() != RobotConstants.EJECT 
    && ((!ActiveUtil.isActive()  && ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE) || ActiveUtil.isActive()) )
    .onTrue( new InstantCommand(() -> Shooter.getInstance().setState(ShooterConstants.WARMUP)));

   T(StateTrigger.T(() -> !SuperStructure.isBalls() || !SuperStructure.isInTheAlinceZone() || ActiveUtil.getTimePastActive() > TIME_PAST_ACTIVE
   , RobotConstants.IDLE_SHOOTER)); //need to put the roller trnacfer etc to idle also


    // Internal sixbar stats
     //TODO need to set ARMBRAKS only on RobotConstants.INTAKE_DEPLOY, on INTAKE_ROLLER its should close it
   new Trigger(() -> getDriverController().getR3() 
    && (getRobotState() == RobotConstants.INTAKE_DEPLOY || getRobotState() == RobotConstants.INTAKE_ROLLER )
    && !SuperStructure.isDefenceMode())
    .onTrue( new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)));
   

    new Trigger(() -> (getDriverController().getR3() && 
    (Math.abs(SixBar.getInstance().getPosition()- SixBarConstants.DEPLOY_ANGLE)) <= SixBarConstants.TOLERANCE_IN_ARM_BRAKE)
    || (!getDriverController().getR1() && (getRobotState() == RobotConstants.INTAKE_DEPLOY || getRobotState() == RobotConstants.INTAKE_ROLLER) // TODO cant work with intake roller
    && SixBar.getInstance().getCurrentState() == SixBarConstants.ARMBRAKS))
    .onTrue( new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.IDLE))); //TODO if the intake it close why dont call the idleIntake?

  new Trigger(() -> (getRobotState() != RobotConstants.IDLE && 
    (SixBar.getInstance().getSetPoint()- SixBar.getInstance().getPosition()) >= SixBarConstants.COLLISION_DETECTION) //TODO abs check
    && (SixBar.getInstance().getCloseLoopVolts() >= SixBarConstants.CLOSE_LOOP_TOLERANCE)) //TODO also abs
    .onTrue( new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.COLLISION)));

  new Trigger(() -> (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION && 
    (SixBar.getInstance().getPosition() - SixBar.getInstance().getSetPoint()) <= SixBarConstants.TOLERANCE))
    .onTrue( new InstantCommand(() -> SixBar.getInstance().setState(SixBar.getInstance().getLastState())));

  new Trigger(() -> (getOperatorController().getActionsDown() ))
  .onTrue( new InstantCommand(() -> SuperStructure.setDefenceMode(!SuperStructure.isDefenceMode())));


  }
}
