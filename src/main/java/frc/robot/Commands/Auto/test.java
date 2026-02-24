

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;


public class test extends SequentialCommandGroup {

  public test() {

    addCommands(
      SwerveAutoFollower.followPath("test")
    );
  }
}
