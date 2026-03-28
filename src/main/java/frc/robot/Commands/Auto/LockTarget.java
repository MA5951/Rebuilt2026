
package frc.robot.Commands.Auto;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.Utils.PIDController;
import com.MAutils.Utils.ChassisSpeedsUtil;
import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class LockTarget extends Command {
  /** Creates a new GoTo. */
  private Pose2d targetPose;
  private PIDController xController = new PIDController(1.5, 0, 0);
  private PIDController yController = new PIDController(1.5, 0, 0);
  private final PIDController tController = new PIDController(0.07, 0, 0).withContinuesInput(-180, 180)
      .withTolerance(3);
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private double atPointRad;
  private boolean stopEnd;
  private double kpMult = 1;

  public LockTarget(Pose2d targetPose, double atPointRad, boolean stopEnd, double kpMult) {
    this.targetPose = targetPose;
    this.atPointRad = atPointRad;
    this.stopEnd = stopEnd;
    this.kpMult = kpMult;

    xController = new PIDController(1.5 * kpMult, 0, 0);
    yController = new PIDController(1.5 * kpMult, 0, 0);

    addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    speeds.vxMetersPerSecond = -xController.calculate(PoseEstimator.getCurrentPose().getX(), targetPose.getX());
    speeds.vyMetersPerSecond = -yController.calculate(PoseEstimator.getCurrentPose().getY(), targetPose.getY());
    speeds.omegaRadiansPerSecond = tController.calculate(Swerve.getInstance().getAbsYaw(),
        SuperStructure.getAbsAngleToTargetFuter());
    MALog.log("/Lock Target/Target Pose", targetPose);
    MALog.log("/Lock Target/Current Pose", PoseEstimator.getCurrentPose());
    MALog.log("/Lock Target/Gyro Yaw", Swerve.getInstance().getGyroYawSupplier().get());

    if (DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue) {
      speeds = ChassisSpeedsUtil.FromFieldToRobot(speeds,
          Rotation2d.fromDegrees(Swerve.getInstance().getGyroYawSupplier().get() - 180));
    } else {
      speeds = ChassisSpeedsUtil.FromFieldToRobot(speeds,
          Rotation2d.fromDegrees(Swerve.getInstance().getGyroYawSupplier().get()));
    }

    MALog.log("/GoTo/Speeds", speeds);

    Swerve.getInstance().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    if (stopEnd) {
      Swerve.getInstance().drive(new ChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    return PoseEstimator.getCurrentPose().getTranslation().getDistance(targetPose.getTranslation()) < atPointRad
        && tController.atSetpoint();
  }
}
