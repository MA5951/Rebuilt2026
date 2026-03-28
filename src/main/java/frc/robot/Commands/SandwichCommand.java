
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;

public class SandwichCommand extends SubsystemCommand {
    private static Sandwich sandwich = Sandwich.getInstance();
    public static Timer startTimer = new Timer();

    public SandwichCommand() {
        //TODO what about the kiker system?
        super(sandwich);
        addRequirements(sandwich);
    }

    @Override
    public void Automatic() {
        switch (sandwich.getCurrentState().stateName) {
            case "IDLE":
                sandwich.setVoltage(SandwichConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                sandwich.setVoltage(SandwichConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                sandwich.setVoltage(SandwichConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION":
                sandwich.setVoltage(SandwichConstants.SHOOTING_VOLTAGE);
                break;
            case "EJECT":
                sandwich.setVoltage(SandwichConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                startTimer.start();
                sandwich.setVoltage(SandwichConstants.SHOOTING_VOLTAGE);
                break;
            case "UNSTUCK":
                sandwich.setVoltage(SandwichConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getL2()) {
            sandwich.setVoltage(10); 
        } else {
            sandwich.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        sandwich.setVoltage(0);
    }

}
