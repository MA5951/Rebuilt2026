
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Kicker.KickerConstants;

public class KickerCommand extends SubsystemCommand {
    private static Kicker kicker = Kicker.getInstance();

    public KickerCommand() {
        //TODO what about the kiker system?
        super(kicker);
        addRequirements(kicker);
    }

    @Override
    public void Automatic() {
        switch (kicker.getCurrentState().stateName) {
            case "IDLE":
                kicker.setVoltage(KickerConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                kicker.setVoltage(KickerConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                kicker.setVoltage(KickerConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION":
                kicker.setVoltage(KickerConstants.SHOOTING_VOLTAGE);
                break;
            case "EJECT":
                kicker.setVoltage(KickerConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                kicker.setVoltage(KickerConstants.SHOOTING_VOLTAGE);
                break;
            case "UNSTUCK":
                kicker.setVoltage(KickerConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getL2()) {
            kicker.setVoltage(6); //TODO move to constants
        } else {
            kicker.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        kicker.setVoltage(0);
    }

}
