
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Commands.SwerveAutoController;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;

public class TwoMagazine extends SequentialCommandGroup {

    public TwoMagazine() {
        addCommands(
                new InstantCommand(() -> RobotConstants.SHOOTING.setState()),
                new ParallelRaceGroup(new SwerveAutoController(), new WaitUntilCommand(1.5)),

                SwerveAutoFollower.followPath("TM1"),
                new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)),
                new InstantCommand(() -> RobotConstants.IDLE_INTAKE.setState()),
                SwerveAutoFollower.followPath("TM2"),
                new InstantCommand(() -> SixBar.getInstance().setState(SixBarConstants.ARMBRAKS)),
                new InstantCommand(() -> RobotConstants.IDLE_INTAKE.setState()),
                SwerveAutoFollower.followPath("TM3"),
                new InstantCommand(() -> RobotConstants.SHOOTING.setState()));
    }
}
