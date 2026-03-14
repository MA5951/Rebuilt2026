
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class MgazineClimbRight extends SequentialCommandGroup {
  public MgazineClimbRight() {
    addCommands(
      SwerveAutoFollower.followPath("MCR1"),
      SwerveAutoFollower.followPath("MCR2"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(2), new SwerveAutoController()),
      SwerveAutoFollower.followPath("MCR3"),
      new InstantCommand(() -> RobotConstants.PRECLIMB.setState()),
      new ParallelDeadlineGroup(new InstantCommand(() -> Climb.getInstance().getIR()), SwerveAutoFollower.followPath("MCR4")),
      new InstantCommand(() -> RobotConstants.CLIMB.setState())
    );
  }
}
