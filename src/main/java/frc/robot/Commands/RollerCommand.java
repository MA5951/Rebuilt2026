
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Roller.RollerConstants;

public class RollerCommand extends SubsystemCommand {
    private static final Roller roller = Roller.getInstance();

    public RollerCommand() {
        super(roller);
        addRequirements(roller);
    }

    @Override
    public void Automatic() {
        switch (roller.getCurrentState().stateName) {
            case "IDLE":
                roller.setVoltage(RollerConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                roller.setVoltage(RollerConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                roller.setVoltage(RollerConstants.FEEDING_VOLTAGE);
                break;
            case "FEDDING_IN_MOTION":
                roller.setVoltage(RollerConstants.FEEDING_IN_MOTION_VOLTAGE);
                break;
            case "EJECT":
                roller.setVoltage(RollerConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                roller.setVoltage(RollerConstants.SHOOTING_VOLTAGE);
                break;
            case "UNSTUCK":
                roller.setVoltage(RollerConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        //TODO need to imploment

    }

    @Override
    public void CantMove() {
        roller.setVoltage(0);
    }

}
