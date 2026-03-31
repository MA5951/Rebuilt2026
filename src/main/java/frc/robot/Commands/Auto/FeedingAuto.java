
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class FeedingAuto extends SequentialCommandGroup {

    public FeedingAuto() {
        addCommands(
                SwerveAutoFollower.followPath("F1")
        );
    }
}
