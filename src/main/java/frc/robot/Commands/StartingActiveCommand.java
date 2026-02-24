
package frc.robot.Commands;

import com.MAutils.Logger.MALog;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class StartingActiveCommand extends SequentialCommandGroup {
  public StartingActiveCommand() {
    addCommands(
      new InstantCommand(() -> MALog.log("/Active util/ start", "starting")),
      new WaitCommand(3),
      new InstantCommand(() -> MALog.log("/Active util/ start", "end")),
      new InstantCommand(() -> RobotContainer.isStartActive = true)
    );
  }
}
