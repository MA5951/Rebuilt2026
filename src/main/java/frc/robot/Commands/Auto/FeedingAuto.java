
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class FeedingAuto extends SequentialCommandGroup {

    public FeedingAuto() {
        addCommands(
                // new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
                // new ParallelRaceGroup(new SwerveAutoController(), new WaitUntilCommand(1.5)),

                SwerveAutoFollower.followPath("F1"),
                new InstantCommand(() -> RobotConstants.FEEDING.setState()),
                new ParallelDeadlineGroup( new WaitCommand(5),SwerveAutoFollower.followPath("F2")),
                new InstantCommand(() -> RobotConstants.FEEDING_IN_MOTION.setState()),
                SwerveAutoFollower.followPath("F3"),
                new InstantCommand(() -> RobotConstants.FEEDING.setState()),
                new WaitCommand(1.5),
                new InstantCommand(() -> RobotConstants.IDLE.setState()),
                SwerveAutoFollower.followPath("F4")

        );
    }
}
