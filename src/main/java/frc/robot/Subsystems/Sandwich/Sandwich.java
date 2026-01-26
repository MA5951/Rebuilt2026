
package frc.robot.Subsystems.Sandwich;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter.Shooter;

public class Sandwich extends PowerControlledSystem {
    private static Sandwich sandwich;

    private Sandwich() {
        super(SandwichConstants.SANDWICH_CONSTANTS, SandwichConstants.IDLE, SandwichConstants.INTAKE, SandwichConstants.FEEDING,
                SandwichConstants.FEDDING_IN_MOTION, SandwichConstants.EJECT,
                SandwichConstants.SHOOTING, SandwichConstants.UNSTUCK);
    }

    @Override
    public void createSelfTest() {
    }

    private boolean canEject() {
        return (Swerve.atPointForSHooting() && hood.atPointForSHooting() && Shooter.atPointForSHooting()) &&
         (RobotContainer.getRobotState() == RobotConstants.SHOOTING || 
         RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS || RobotContainer.getRobotState() == RobotConstants.FEEDING);//TODO After Merge
    }

    private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY || 
         RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallDetected();

    private boolean canEject() {
        return (Swerve.atPoint() && hood.atPoint() && Shooter.atVelocity()) &&
         (RobotContainer.getRobotState() == RobotConstants.SHOOTING || 
         RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS || RobotContainer.getRobotState() == RobotConstants.FEEDING);//TODO After Merge
    }

    @Override
    public boolean CAN_MOVE() {
        return canEject() || canIntake()|| RobotContainer.getRobotState() == RobotConstants.UNSTUCK  
        || RobotContainer.getRobotState() == RobotConstants.EJECT
        || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE 
        || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
        || RobotContainer.getRobotState() == RobotConstants.IDLE 
        || (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION && hood.atPointForFeeding() && Shooter.atPointForFeeding && SuperStructer.canFeeedingInMotion());//TODO After Merge


    }

    public static Sandwich getInstance() {
        if (sandwich == null) {
            sandwich = new Sandwich();
        }
        return sandwich;
    }
}