package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Util.Field;

public class ShootingClimbL extends SequentialCommandGroup {
  public ShootingClimbL() {
    addCommands(
      SwerveAutoFollower.followPath("F4"),
      new InstantCommand(() -> RobotConstants.SHOOTING_UNLOCKED.setState()),
      new ParallelCommandGroup(new LockTarget(Field.flipByAlliance(new Pose2d(1.067,2.454,new Rotation2d())), 0.1, true, 0.3), new WaitCommand(6)),
      new InstantCommand(() -> RobotConstants.PRECLIMB.setState()),
      new GoTo(Field.flipByAlliance(new Pose2d(1.067,2.454, Rotation2d.fromDegrees(90))), 0.07, true),
      new ParallelDeadlineGroup(new SequentialCommandGroup(
      new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) > 0.1),
      new WaitCommand(1),
      new WaitUntilCommand(() -> Math.abs(Swerve.getInstance().getCurrentStates()[1].speedMetersPerSecond) < 0.3)), 
      new Drive(0.5, 0, 0)),
      new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().getIR()), new Drive(0, -0.2, 0)),
      new InstantCommand(() -> RobotConstants.CLIMB.setState())
    );
  }
}
