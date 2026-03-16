
package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Util.Field;

public class TwoMagazine extends SequentialCommandGroup {

    public TwoMagazine() {

        addCommands(
                SwerveAutoFollower.followPath("TM5"),
                new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
                new ParallelDeadlineGroup(new WaitCommand(5), new SwerveAutoController()),
                new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()),
                new GoTo(Field.flipByAlliance(new Pose2d(0.416, 5.925, Rotation2d.fromDegrees(-90))), 0.15, false),
                SwerveAutoFollower.followPath("TM6"),
                new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
                new ParallelDeadlineGroup(new WaitCommand(5), new SwerveAutoController()));

        // addCommands(
        //         SwerveAutoFollower.followPath("TM5"),
        //         new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
        //         new ParallelDeadlineGroup(new WaitCommand(5), new SwerveAutoController()),
        //         new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()),
        //         SwerveAutoFollower.findfollowPath("TM4"),
        //         new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
        //         new ParallelDeadlineGroup(new WaitCommand(5), new SwerveAutoController()));
    }
}
