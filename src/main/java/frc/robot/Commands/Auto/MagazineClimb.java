
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class MagazineClimb extends SequentialCommandGroup {
  public MagazineClimb() {
    addCommands(
      SwerveAutoFollower.followPath("MC1"),
      new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)),
      new InstantCommand(() -> RobotConstants.IDLE_INTAKE.setState()),
      SwerveAutoFollower.followPath("MC2"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(2), new SwerveAutoController()),
      new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()),
      SwerveAutoFollower.followPath("MC3")
    );
  }
}
