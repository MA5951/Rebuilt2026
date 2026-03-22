
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class FeedingBack extends SequentialCommandGroup {

  public FeedingBack() {

    addCommands(
      SwerveAutoFollower.followPath("FB1"),
      SwerveAutoFollower.followPath("FB2")
      
    );
  }
}
