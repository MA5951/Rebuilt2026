
package frc.robot.Commands;

import com.MAutils.Logger.MALog;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class StartingActiveCommand extends SequentialCommandGroup {
  public StartingActiveCommand() {
    addCommands(
      new InstantCommand(() -> RobotContainer.driverRumble.setRumble(RumbleType.kBothRumble, 0.3)),
      new WaitCommand(1.5),
      new InstantCommand(() -> RobotContainer.driverRumble.setRumble(RumbleType.kBothRumble, 0))
    );
  }
}
