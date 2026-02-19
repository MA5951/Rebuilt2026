
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class StartingActiveCommand extends SequentialCommandGroup {
  public StartingActiveCommand() {
    addCommands(
      new InstantCommand(() -> RobotContainer.getDriverController().setRumble(1)),
      new WaitCommand(3),
      new InstantCommand(() -> RobotContainer.getDriverController().setRumble(0)),
      new InstantCommand(() -> RobotContainer.isStartActive = true)
    );
  }
}
