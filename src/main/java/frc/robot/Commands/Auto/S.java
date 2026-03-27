
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class S extends SequentialCommandGroup {
  public S() {
    addCommands(
      SwerveAutoFollower.followPath("S3"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(8), new SwerveAutoController()),
      new InstantCommand(()-> RobotConstants.IDLE_INTAKE.setState()),
      SwerveAutoFollower.followPath("S2")
      
    );
  }
}
