
package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class RobotConstants {

  
    public static final RobotState IDLE = new RobotState("IDLE",
            IntakeRollerConstants.IDLE, SixBarConstants.IDLE, ShooterConstants.IDLE, SandwichConstants.IDLE, );
}
