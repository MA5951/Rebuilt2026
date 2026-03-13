package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class FeedingClimb extends SequentialCommandGroup {
  public FeedingClimb() {
    addCommands(
      SwerveAutoFollower.followPath("FC1"),
      SwerveAutoFollower.followPath("FC2"),
      new InstantCommand(() -> RobotConstants.PRECLIMB.setState()),
      new ParallelDeadlineGroup(new InstantCommand(() -> Climb.getInstance().getIR()), SwerveAutoFollower.followPath("FC3")),
      new InstantCommand(() -> RobotConstants.CLIMB.setState())
    );
  }
}
