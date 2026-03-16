
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class FeedingShooting extends SequentialCommandGroup {
  public FeedingShooting() {
    addCommands(
      SwerveAutoFollower.followPath("FS1")
    );
  }
}
