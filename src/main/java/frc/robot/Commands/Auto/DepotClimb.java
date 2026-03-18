
package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Util.Field;

public class DepotClimb extends SequentialCommandGroup {
  public DepotClimb() {
    addCommands(
      SwerveAutoFollower.followPath("DC 1 Push"),
      new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
      new ParallelDeadlineGroup(new WaitCommand(6.5), new SwerveAutoController()),
      new InstantCommand(() -> RobotConstants.PRECLIMB.setState()),
      new GoTo(Field.flipByAlliance(new Pose2d(1.145,4.227, Rotation2d.fromDegrees(-90))) , 0.15, false),
      new ParallelDeadlineGroup(new SequentialCommandGroup(
      new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) > 0.05),
      new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) < 0.1)
    ), new Drive(0.5, -0.1, 0)),
    new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().getIR()), new Drive(0, -0.2, 0)),
    new InstantCommand(() -> RobotConstants.CLIMB.setState())
    );
  }
}
