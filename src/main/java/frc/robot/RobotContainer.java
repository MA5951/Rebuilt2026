
package frc.robot;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.RobotControl.StateTrigger;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.HoodCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.KickerCommand;
import frc.robot.Commands.RollerCommand;
import frc.robot.Commands.SandwichCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.SixBarCommand;
import frc.robot.Commands.StartingActiveCommand;
import frc.robot.Commands.SwerveController;
import frc.robot.Commands.TransferCommand;
import frc.robot.Commands.Auto.FeedingAuto;
import frc.robot.Commands.Auto.test;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.ActiveUtil;

public class RobotContainer extends DeafultRobotContainer {

    public static final double TIME_UNTIL_ACTIVE = 1;
    public static final double TIME_PAST_ACTIVE = 1;
    public static boolean isStartActive = false;

    public RobotContainer() {
        super();

        
        CommandScheduler.getInstance().setDefaultCommand(Swerve.getInstance(), new SwerveController());
        CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterCommand());
        CommandScheduler.getInstance().setDefaultCommand(SixBar.getInstance(), new SixBarCommand());
        SixBar.getInstance();
        CommandScheduler.getInstance().setDefaultCommand(Sandwich.getInstance(), new SandwichCommand());
        CommandScheduler.getInstance().setDefaultCommand(Transfer.getInstance(), new TransferCommand());
        Transfer.getInstance();
        CommandScheduler.getInstance().setDefaultCommand(Hood.getInstance(), new HoodCommand());
        CommandScheduler.getInstance().setDefaultCommand(Roller.getInstance(), new RollerCommand());
        CommandScheduler.getInstance().setDefaultCommand(Kicker.getInstance(), new KickerCommand());
        CommandScheduler.getInstance().setDefaultCommand(IntakeRoller.getInstance(),
                new IntakeCommand());
        IntakeRoller.getInstance();
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbCommand());

        
        

    }
 
    @Override
    public void configAuto() {
        new SwerveAutoFollower();
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()));
        NamedCommands.registerCommand("Shooting", new InstantCommand(() -> RobotConstants.SHOOTING.setState()));
        NamedCommands.registerCommand("Feeding", new InstantCommand(() -> RobotConstants.FEEDING.setState()));
        NamedCommands.registerCommand("FeedingInMotion", new InstantCommand(() -> RobotConstants.FEEDING_IN_MOTION.setState()));


        // new Trigger(() -> PoseEstimator.getCurrentPose().getX() > 5.5 && DriverStation.isAutonomous()//TODO Handel Red Side
        // ).onTrue(new InstantCommand(() -> RobotConstants.FEEDING_IN_MOTION.setState()));

        new Trigger(() -> PoseEstimator.getCurrentPose().getX() > 5.5 && DriverStation.isAutonomous()//TODO Handel Red Side
        ).onTrue(new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()));
        
    }

    @Override
    public void configBinding() {
        new Trigger(() -> getDriverController().getActionsUp()).onTrue(
                new InstantCommand(() -> SwerveConstants.FIELD_CENTRIC_DRIVE.updateOffset()));

        new Trigger(() -> getDriverController().getOptionsRight()).onTrue(
                new InstantCommand(() -> SuperStructure.setAutomatic(!SuperStructure.isAutomatic())));

        T(StateTrigger.T(() -> getDriverController().getMiddle(), RobotConstants.IDLE));

        T(StateTrigger.T(
                () -> ((getRobotState() == RobotConstants.INTAKE_DEPLOY
                        || getRobotState() == RobotConstants.INTAKE_ROLLER)
                        && ((!getDriverController().getR1()) || SuperStructure.isFull() ) && !DriverStation.isAutonomous() ) ||//&& !DriverStation.isTeleop()
                        (getRobotState() == RobotConstants.EJECT && (!getDriverController().getActionsRight() && !DriverStation.isAutonomous() ))//&& !DriverStation.isTeleop()
                        || (getRobotState() == RobotConstants.FEEDING
                                && ((!getDriverController().getR2() || !SuperStructure.isBalls()) && !DriverStation.isAutonomous() ))//&& !DriverStation.isTeleop()
                        || (getRobotState() == RobotConstants.FEEDING_IN_MOTION && (!getDriverController().getR2() && !DriverStation.isAutonomous() ))//&& !DriverStation.isTeleop()
                        || (getRobotState() == RobotConstants.SHOOTING
                                && ((!getDriverController().getL1() || !SuperStructure.isBalls()
                                        ) && !DriverStation.isAutonomous() ))//&& !DriverStation.isTeleop()

                                        //|| (!ActiveUtil.isActive()
                                        //        && ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE)
                        || getRobotState() == RobotConstants.SHOOTING_PRESETS && !getDriverController().getL1(),
                RobotConstants.IDLE_INTAKE));

        T(StateTrigger.T(
                () -> getDriverController().getR1()
                        && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS,
                RobotConstants.INTAKE_DEPLOY));

        T(StateTrigger.T(
                () -> getDriverController().getR1()
                        && SixBar.getInstance().getCurrentState() == SixBarConstants.ARMBRAKS,
                RobotConstants.INTAKE_ROLLER));

        //&& (ActiveUtil.isActive() || ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE
                          //      || ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE)&& (ActiveUtil.isActive() || ActiveUtil.getTimePastActive() < TIME_PAST_ACTIVE
                           //     || ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE)

        T(StateTrigger.T(
                () -> getDriverController().getL1()
                        
                        &&
                        getRobotState() != RobotConstants.UNSTUCK && SuperStructure.isAutomatic(),
                RobotConstants.SHOOTING));

        T(StateTrigger.T(() -> getDriverController().getActionsRight() && getRobotState() != RobotConstants.UNSTUCK,
                RobotConstants.EJECT));

        T(StateTrigger.T(() -> getDriverController().getR2() && Swerve.getInstance().getVelocityVector() < 0.2 && getDriverController().inDeadbound(),
                RobotConstants.FEEDING));

        T(StateTrigger.T(() -> getDriverController().getR2() && ! getDriverController().inDeadbound(),
                RobotConstants.FEEDING_IN_MOTION));

        T(StateTrigger.T(
                () -> getDriverController().getL1() && !SuperStructure.isAutomatic()
                        && getRobotState() != RobotConstants.UNSTUCK,
                RobotConstants.SHOOTING_PRESETS));

        T(StateTrigger.T(
                () -> (getRobotState() == RobotConstants.SHOOTING || getRobotState() == RobotConstants.SHOOTING_PRESETS
                        || getRobotState() == RobotConstants.EJECT || getRobotState() == RobotConstants.FEEDING
                        || getRobotState() == RobotConstants.FEEDING_IN_MOTION) && SuperStructure.isTransferStuck(),
                RobotConstants.UNSTUCK));

        T(StateTrigger.T(() -> getRobotState() == RobotConstants.UNSTUCK && !SuperStructure.isTransferStuck(),
                getLastRobotState()));

        T(StateTrigger.T(() -> getDriverController().getActionsLeft() 
                &&
                Climb.getInstance().getPosition() < 0.05, RobotConstants.PRECLIMB));

        T(StateTrigger.T(() -> getDriverController().getActionsLeft()
                &&
                ClimbCommand.isAtPosition, RobotConstants.CLIMB));

       

        new Trigger(() -> SuperStructure.isBalls() && SuperStructure.isInWarmUpZone()
                && getRobotState() != RobotConstants.SHOOTING && getRobotState() != RobotConstants.SHOOTING_PRESETS
                && getRobotState() != RobotConstants.FEEDING && getRobotState() != RobotConstants.FEEDING_IN_MOTION
                && getRobotState() != RobotConstants.EJECT && getRobotState() != RobotConstants.PRECLIMB
                && getRobotState() != RobotConstants.CLIMB)
                
                .onTrue(new InstantCommand(() -> Shooter.getInstance().setState(ShooterConstants.WARMUP)));

                // && ((!ActiveUtil.isActive() && ActiveUtil.getTimeUntilActive() < TIME_UNTIL_ACTIVE)
                //         || ActiveUtil.isActive()))

                // || (ActiveUtil.getTimePastActive() > TIME_PAST_ACTIVE)
                // || (getLastRobotState() == RobotConstants.EJECT && !getDriverController().getActionsRight())

        new Trigger(() -> (((!SuperStructure.isBalls()) || (!SuperStructure.isInWarmUpZone())
                ))
                && getRobotState() != RobotConstants.SHOOTING
                && getRobotState() != RobotConstants.SHOOTING_PRESETS
                && getRobotState() != RobotConstants.FEEDING && getRobotState() != RobotConstants.FEEDING_IN_MOTION
                && getRobotState() != RobotConstants.EJECT)
                .onTrue(new InstantCommand(() -> Shooter.getInstance().setState(ShooterConstants.IDLE)));

        // Internal sixbar stats
        new Trigger(() -> (getRobotState() == RobotConstants.INTAKE_DEPLOY))
                .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)));

        new Trigger(() -> (((getDriverController().getActionsDown() && SixBar.getInstance().atPoint(10))
                && getRobotState() != RobotConstants.INTAKE_DEPLOY && getRobotState() != RobotConstants.INTAKE_ROLLER)
                || (!getDriverController().getR1()
                        && (getLastRobotState() == RobotConstants.INTAKE_DEPLOY
                                || getLastRobotState() == RobotConstants.INTAKE_ROLLER)
                        && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS)
                || (!getDriverController().getR2()
                        && getLastRobotState() == RobotConstants.FEEDING_IN_MOTION)

                        && SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS)
                && SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION)
                .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.IDLE)));

        new Trigger(() -> (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION
                && SixBar.getInstance().getPosition() < -60))
                .onTrue(new InstantCommand(() -> SixBar.getInstance().setState(SixBar.getInstance().getLastState())));

        new Trigger(() -> (getOperatorController().getActionsDown()))
                .onTrue(new InstantCommand(() -> SuperStructure.setDefenceMode(!SuperStructure.isDefenceMode())));

        new Trigger(() -> getDriverController().getDpadDown()).onTrue(
                new InstantCommand(() -> PoseEstimator.resetPose(VisionConstants.FRONT_LL.getCameraIO().getPoseEstimate(PoseEstimateType.MT1).pose))
        );

        new Trigger(() -> ActiveUtil.isActive() && !isStartActive).onTrue(new StartingActiveCommand());

        new Trigger(() -> getDriverController().getDpadLeft()).onTrue(new InstantCommand(() -> MALog.log("Flags/Align Flag", Timer.getFPGATimestamp())));

        new Trigger(() -> getDriverController().getDpadRight()).onTrue(new InstantCommand(() -> MALog.log("Flags/Miss Flag", Timer.getFPGATimestamp())));

        
        new Trigger(() -> !ActiveUtil.isActive() && isStartActive).onTrue(new InstantCommand(() -> isStartActive = false));

        new Trigger (() -> getOperatorController().getActionsRight()).onTrue
        (new InstantCommand(() -> SixBar.getInstance().setState(SixBar.HOMING)));

        new Trigger (() -> getOperatorController().getActionsLeft()).onTrue
        (new InstantCommand(() -> Hood.getInstance().setState(Hood.HOMING)));

        new Trigger (() -> getOperatorController().getDpadRight()).onTrue
        (new InstantCommand(() -> Climb.getInstance().setState(Climb.HOMING)));

         new Trigger (() -> getOperatorController().getL1()).onTrue
        (new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.FORCE_OPEN)));

        new Trigger (() -> getOperatorController().getR1()).onTrue
        (new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.FORCE_CLOSE)));

        new Trigger (() -> getOperatorController().getDpadLeft()).onTrue
        (new InstantCommand(() -> SuperStructure.SetIsExtendedMagazine(!SuperStructure.isExtendedMagazine())));

        new Trigger (() -> getOperatorController().getR2()).onTrue
        (new InstantCommand(() -> Climb.getInstance().setState(ClimbConstnats.IDLE)));
        
    }
}
