
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
            || (getRobotState() == RobotConstants.FEEDING
                && (!getDriverController().getActionsLeft() || !SuperStructure.isBalls()))
            || (getRobotState() == RobotConstants.FEEDING_IN_MOTION && !getDriverController().getR2())
            || (getRobotState() == RobotConstants.SHOOTING
                && (!getDriverController().getL1() || !SuperStructure.isBalls()
                    || (!ActiveUtil.isActive() && ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE)))
            || getRobotState() == RobotConstants.SHOOTING_PRESETS && !getDriverController().getR2(),
        RobotConstants.IDLE_INTAKE));

    T(StateTrigger.T(
        () -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_DEPLOY));

    T(StateTrigger.T(
        () -> getDriverController().getR1() && SixBar.getInstance().getCurrentState() == SixBarConstants.ARMBRAKS,
        RobotConstants.INTAKE_ROLLER));

    T(StateTrigger.T(
        () -> getDriverController().getL1()
            && (ActiveUtil.isActive() || ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE
                || ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE)
            &&
            getRobotState() != RobotConstants.UNSTUCK && SuperStructure.isAutomatic(),
        RobotConstants.SHOOTING));

    T(StateTrigger.T(() -> getDriverController().getActionsRight() && getRobotState() != RobotConstants.UNSTUCK,
        RobotConstants.EJECT));

    T(StateTrigger.T(() -> getDriverController().getActionsLeft() && getRobotState() != RobotConstants.UNSTUCK,
        RobotConstants.FEEDING));

    T(StateTrigger.T(() -> getDriverController().getR2() && getRobotState() != RobotConstants.UNSTUCK,
        RobotConstants.FEEDING_IN_MOTION));

    T(StateTrigger.T(
        () -> getDriverController().getL2() && !SuperStructure.isAutomatic()
            && getRobotState() != RobotConstants.UNSTUCK,
        RobotConstants.SHOOTING_PRESETS));

    T(StateTrigger.T(
        () -> (getRobotState() == RobotConstants.SHOOTING || getRobotState() == RobotConstants.SHOOTING_PRESETS
            || getRobotState() == RobotConstants.EJECT || getRobotState() == RobotConstants.FEEDING
            || getRobotState() == RobotConstants.FEEDING_IN_MOTION) && SuperStructure.isTransferStuck(),
        RobotConstants.UNSTUCK));

    T(StateTrigger.T(() -> getRobotState() == RobotConstants.UNSTUCK && !SuperStructure.isTransferStuck(),
        getLastRobotState()));

    T(StateTrigger.T(() -> getDriverController().getActionsLeft() &&
        !SuperStructure.isRobotInAir()
        && Math.abs(
            Climb.getInstance().getPosition() - ClimbConstnats.CLOSE_POSITION) < ClimbConstnats.TOLERANCE_FOR_OPENCLOSE_TRIGGER
        && SuperStructure.getTimeLeft() < 30, RobotConstants.PRECLIMB));

    T(StateTrigger.T(() -> getDriverController().getActionsLeft()
        && Math.abs(
            Climb.getInstance().getPosition() - ClimbConstnats.OPEN_POSITION) < ClimbConstnats.TOLERANCE_FOR_OPENCLOSE_TRIGGER
        && SuperStructure.getTimeLeft() < 30, RobotConstants.CLIMB));

    // Internal climb stats
    new Trigger(() -> getDriverController().getActionsLeft()
        && SuperStructure.getTimeLeft() < 30 && SuperStructure.isRobotInAir()
        && (Math.abs(
            Climb.getInstance().getPosition() - ClimbConstnats.CLOSE_POSITION) < ClimbConstnats.TOLERANCE_FOR_OPENCLOSE_TRIGGER
            || SuperStructure.getTimeLeft() > 20))
        .onTrue(new InstantCommand(() -> Climb.getInstance().setState(ClimbConstnats.DOWN)));

    // Internal shooter stats

    new Trigger(() -> SuperStructure.isBalls() && SuperStructure.isInTheAlinceZone()
        && getRobotState() != RobotConstants.SHOOTING && getRobotState() != RobotConstants.SHOOTING_PRESETS
        && getRobotState() != RobotConstants.FEEDING && getRobotState() != RobotConstants.FEEDING_IN_MOTION
        && getRobotState() != RobotConstants.EJECT && getRobotState() != RobotConstants.PRECLIMB
        && getRobotState() != RobotConstants.CLIMB
        && ((!ActiveUtil.isActive() && ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE) || ActiveUtil.isActive()))
        .onTrue(new InstantCommand(() -> Shooter.getInstance().setState(ShooterConstants.WARMUP)));

    T(StateTrigger.T(() -> !SuperStructure.isBalls() || !SuperStructure.isInTheAlinceZone()
        || ActiveUtil.getTimePastActive() > TIME_PAST_ACTIVE
        , RobotConstants.IDLE_SHOOTER));

    // Internal sixbar stats
    new Trigger(() -> getDriverController().getR3()
        && (getRobotState() == RobotConstants.INTAKE_DEPLOY))
        .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)));

    new Trigger(() -> (getDriverController().getR3() &&
        (Math.abs(SixBar.getInstance().getPosition()
            - SixBarConstants.DEPLOY_ANGLE)) <= SixBarConstants.TOLERANCE_IN_ARM_BRAKE)
        || (!getDriverController().getR1()
            && (getRobotState() == RobotConstants.INTAKE_DEPLOY || getRobotState() == RobotConstants.INTAKE_ROLLER)
            && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS))
        .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.IDLE)));

    new Trigger(() -> (getRobotState() != RobotConstants.IDLE &&
        Math.abs(SixBarConstants.DEPLOY_ANGLE
            - SixBar.getInstance().getPosition()) >= SixBarConstants.COLLISION_DETECTION)
        && (SixBar.getInstance().getCloseLoopVolts() >= SixBarConstants.CLOSE_LOOP_TOLERANCE))
        .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.COLLISION)));

    new Trigger(() -> (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION &&
        (SixBar.getInstance().getPosition() - SixBar.getInstance().getSetPoint()) <= SixBarConstants.TOLERANCE))
        .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBar.getInstance().getLastState())));

    new Trigger(() -> (getOperatorController().getActionsDown()))
        .onTrue(new InstantCommand(() -> SuperStructure.setDefenceMode(!SuperStructure.isDefenceMode())));

  }
}
