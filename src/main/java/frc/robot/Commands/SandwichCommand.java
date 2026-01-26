
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;

public class SandwichCommand extends SubsystemCommand {
    private static Sandwich sandwich = Sandwich.getInstance();

    public SandwichCommand() {
        super(sandwich);
        addRequirements(sandwich);
    }

    @Override
    public void Automatic() {
        switch (sandwich.getCurrentState().stateName) {
            case "IDLE":
                sandwich.setVoltage(SandwichConstants.IDLE_VOLTAGE);
                break;
        
            case "INTAKE" :
                sandwich.setVoltage(SandwichConstants.INTAKE_VOLTAGE);
                break;
            
            case "FEEDING" :
                sandwich.setVoltage(SandwichConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION" :
                sandwich.setVoltage(SandwichConstants.FEDDING_IN_MOTION_VOLTAGE);
                break;
            case "FEEDING_EJECT" :
                sandwich.setVoltage(SandwichConstants.FEEDING_EJECT_VOLTAGE);
                break;
            case "SHOOTING" :
                sandwich.setVoltage(SandwichConstants.SHOOTING_VOLTAGE);
                break;
            case "UNSTUCK" :
                sandwich.setVoltage(SandwichConstants.UNSTUCK_VOLTAGE); 
                break;
        }
    }

    @Override
    public void Manual() {
        
    }

    @Override
    public void CantMove() {
        sandwich.setVoltage(0);
    }

}
