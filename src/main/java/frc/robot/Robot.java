
package frc.robot;

import com.MAutils.DashBoard.DashBoard;
import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobot;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.SixBarCommand;
import frc.robot.Commands.SwerveController;
import frc.robot.Commands.Auto.DepotClimb;
import frc.robot.Commands.Auto.Drive;
import frc.robot.Commands.Auto.FeedingAuto;
import frc.robot.Commands.Auto.GoTo;
import frc.robot.Commands.Auto.MagazineClimb;
import frc.robot.Commands.Auto.S;
import frc.robot.Commands.Auto.TwoMagazine;
import frc.robot.RobotControl.Dashboard;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.ActiveUtil;
import frc.robot.Util.Field;
import frc.robot.lib.MatchTime;

public class Robot extends DeafultRobot {

  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;
  public static int counter = 0;

  private Command auto;

  @Logged(name = "MatchTime")
  private final MatchTime matchTime = new MatchTime(2026);

  public Robot() {
    super();
    Vision.getInstance();

    m_robotContainer = new RobotContainer();
    PoseEstimator.resetPose(new Pose2d(Field.LENGTH / 2, Field.WIDTH / 2, new Rotation2d()));

    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();

    auto = new S();
    //auto = new DepotClimb();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    MALog.log("/RobotControl/Current RobotState", RobotContainer.getRobotState().getStateName());
    MALog.log("/RobotControl/Last RobotState", RobotContainer.getLastRobotState().getStateName());
    MALog.log("/RobotControl/Test", (((!SuperStructure.isBalls()) || (!SuperStructure.isInTheAlinceZone())
        || (ActiveUtil.getTimePastActive() > RobotContainer.TIME_PAST_ACTIVE)
        || (RobotContainer.getLastRobotState() == RobotConstants.EJECT
            && !RobotContainer.getDriverController().getActionsRight())))
        && RobotContainer.getRobotState() != RobotConstants.SHOOTING
        && RobotContainer.getRobotState() != RobotConstants.SHOOTING_PRESETS
        && RobotContainer.getRobotState() != RobotConstants.FEEDING
        && RobotContainer.getRobotState() != RobotConstants.FEEDING_IN_MOTION
        && RobotContainer.getRobotState() != RobotConstants.EJECT);
    SuperStructure.update();
    MALog.log("/SuperStructure/Intake Stuck", (RobotContainer.getRobotState() != RobotConstants.IDLE &&
        (Math.abs(SixBar.getInstance().getPosition())
            - SixBarConstants.DEPLOY_ANGLE) >= SixBarConstants.COLLISION_DETECTION));
    MALog.log("/SuperStructure/Intake Tolorance",
        Math.abs(Math.abs(SixBar.getInstance().getPosition()) - SixBarConstants.DEPLOY_ANGLE));

    if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && (SwerveController.isAbs < 3)
        
            && (Vision.getInstance().isTagInFrame(24) ||
                Vision.getInstance().isTagInFrame(27) || Vision.getInstance().isTagInFrame(8) || Vision.getInstance().isTagInFrame(11))) {



      Vision.getInstance().filterCornerTags();
    } else if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && (SwerveController.isAbs < 3)) {
      Vision.getInstance().filterHubTags();
    } else {
      Vision.getInstance().resetFilter();
    }

    if ((Swerve.getInstance().isRampFlag() || PoseEstimator.isOutOfFieldFlag()) && VisionConstants.FRONT_LL.getCameraIO().isTag()) {
      Swerve.getInstance().resetRampFlag();
      PoseEstimator.resetOutOfFieldFlag();
      PoseEstimator.resetPose(VisionConstants.FRONT_LL.getCameraIO().getPoseEstimate(PoseEstimateType.MT1).pose);
    }

    matchTime.update(MatchTime.kGameData2026.get());

  }

  @Override
  public void autonomousInit() {
    super.autonomousInit();
    if (!SixBarCommand.isReset) {
      SixBar.getInstance().setState(SixBar.HOMING);
      Hood.getInstance().setState(Hood.HOMING);
    }

   

    CommandScheduler.getInstance().schedule(auto);
    // CommandScheduler.getInstance().schedule(new GoTo(Field.flipByAlliance(new Pose2d(1.445,4.627, Rotation2d.fromDegrees(-90))) , 0.1, true));
    // CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
    //   new InstantCommand(() -> RobotConstants.PRECLIMB.setState()),
    //   new GoTo(Field.flipByAlliance(new Pose2d(1.145,4.227, Rotation2d.fromDegrees(-90))) , 0.15, false),
    //   new ParallelDeadlineGroup(new SequentialCommandGroup(
    //   new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) > 0.05),
    //   new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) < 0.1)
    // ), new Drive(0.5, -0.1, 0)),
    // new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().getIR()), new Drive(0, -0.2, 0)),
    // new InstantCommand(() -> RobotConstants.CLIMB.setState())
    // ));
  }

  @Override 
  public void autonomousExit() {
    Climb.getInstance().setBrakeMode(false);
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    ActiveUtil.startTeleop();

    CommandScheduler.getInstance().setDefaultCommand(Swerve.getInstance(), new SwerveController());

    if (!SixBarCommand.isReset) {
      SixBar.getInstance().setState(SixBar.HOMING);
      Hood.getInstance().setState(Hood.HOMING);
    }

   

  }

  @Override
  public void teleopPeriodic() {
    super.teleopPeriodic();
    ActiveUtil.getGameMode();
    MALog.log("ActiveUtil/is active", ActiveUtil.isActive());

    if (ActiveUtil.isActive()) {
      MALog.log("ActiveUtil/Active Time", ActiveUtil.getTimeInActive());
    } else {
      MALog.log("ActiveUtil/Active Time", ActiveUtil.getTimeUntilActive());
    }

    

  }

  @Override
  public void teleopExit() {
    super.teleopExit();
  }

  @Override
  public void disabledInit() {
    super.disabledInit();
    Swerve.getInstance().drive(new ChassisSpeeds());
  }

}
