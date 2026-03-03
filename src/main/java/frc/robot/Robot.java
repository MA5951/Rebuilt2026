
package frc.robot;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobot;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Commands.SixBarCommand;
import frc.robot.Commands.SwerveController;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.ActiveUtil;
import frc.robot.Util.Field;

public class Robot extends DeafultRobot {

  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;
  public static int counter = 0;

  public Robot() {
    super();
    Vision.getInstance();
    
    m_robotContainer = new RobotContainer();
    PoseEstimator.resetPose(new Pose2d(Field.LENGTH / 2, Field.WIDTH / 2, new Rotation2d()));
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
        
        


        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && (SwerveController.isAbs < 3)) {
          Vision.getInstance().filterMainTag();
          counter++;
        } else {
          Vision.getInstance().resetFilter();
          
        }

        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && (SwerveController.isAbs < 3) && counter > 10 && VisionConstants.FRONT_LL.getCameraIO().getFiducials().length > 1) {
          Vision.getInstance().filterCenterMainTag();
        }

        if (Swerve.getInstance().isRampFlag() && VisionConstants.FRONT_LL.getCameraIO().isTag()) {
          Swerve.getInstance().resetRampFlag();
          PoseEstimator.resetPose(VisionConstants.FRONT_LL.getCameraIO().getPoseEstimate(PoseEstimateType.MT1).pose);
        }

  }

  @Override
  public void autonomousInit() {
    super.autonomousInit();
    if (!SixBarCommand.isReset) {
      SixBar.getInstance().setState(SixBar.HOMING);
    }
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    ActiveUtil.startTeleop();
    if (!SixBarCommand.isReset) {
      SixBar.getInstance().setState(SixBar.HOMING);
    }

    if (SuperStructure.isRobotInAir()) {
      Climb.getInstance().setState(ClimbConstnats.DOWN);
    }
  }

  @Override
  public void teleopPeriodic() {
    super.teleopPeriodic();
    ActiveUtil.checkShift();

  }

  @Override
  public void teleopExit() {
    super.teleopExit();
    Climb.getInstance().setBrakeMode(true);
  }

}
