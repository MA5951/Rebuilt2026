
package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Util.Field;

public class FeedingAuto extends SequentialCommandGroup {

    public FeedingAuto() {
        addCommands(
                SwerveAutoFollower.followPath("F2"),
                new InstantCommand(() -> RobotConstants.SHOOTING_UNLOCKED.setState()),
                new LockTarget(Field.flipByAlliance(new Pose2d(3.121,0.89,new Rotation2d())), 0.1, true, 0.3)
        );
    }
}
