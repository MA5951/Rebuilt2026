
package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Util.Field;

public class S extends SequentialCommandGroup {
  public S() {
    addCommands(
      SwerveAutoFollower.followPath("S3"),
      new InstantCommand(() -> RobotConstants.SHOOTING_UNLOCKED.setState()),
  
      new ParallelDeadlineGroup(new WaitCommand(9), new LockTarget(Field.flipByAlliance(new Pose2d(2,5.219,new Rotation2d())), 0.1, true, 0.3)),
      new LockTarget(Field.flipByAlliance(new Pose2d(3.427,5.219,new Rotation2d())), 0.5, false, 0.3),
      // new ParallelDeadlineGroup(new WaitCommand(8), new SwerveAutoController()),
      SwerveAutoFollower.followPath("S2")
      
    );
  }
}
