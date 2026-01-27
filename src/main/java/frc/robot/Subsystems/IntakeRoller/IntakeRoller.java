
package frc.robot.Subsystems.IntakeRoller;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;

public class IntakeRoller extends PowerControlledSystem{
    private static IntakeRoller intakeroller;

    private IntakeRoller() {
        super(IntakeRollerConstants.INTAKE_ROLLER_CONSTANTS, IntakeRollerConstants.IDLE,
         IntakeRollerConstants.FORWARD, IntakeRollerConstants.BACKWARD);
    }

    @Override
    public boolean CAN_MOVE() { // TODO After Merge SuperStructure
        return !SuperStracter.isMagazinFull() && (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY 
        || RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER);
    }

    @Override
    public void createSelfTest() {
        
    }

    public static IntakeRoller getInstance() {
        if (intakeroller == null) {
            intakeroller = new IntakeRoller();
        }
        return intakeroller;
    }

    
}
