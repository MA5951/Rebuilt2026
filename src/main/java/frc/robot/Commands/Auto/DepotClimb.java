
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class DepotClimb extends SequentialCommandGroup {
  public DepotClimb() {
    addCommands(
      SwerveAutoFollower.followPath("DC 1"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(2), new SwerveAutoController()),
      new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()),
      SwerveAutoFollower.followPath("DC 2"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(2), new SwerveAutoController()),
      new InstantCommand(() -> RobotConstants.PRECLIMB.setState())
    );
  }
}
