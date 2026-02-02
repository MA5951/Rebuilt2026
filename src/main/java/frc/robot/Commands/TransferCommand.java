
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class TransferCommand extends SubsystemCommand {
    private static Transfer transfer = Transfer.getInstance();

    public TransferCommand() {
        super(transfer);
        addRequirements(transfer);
    }

    @Override
    public void Automatic() {
        //TODO what about the pulse imoplomation? base on num of balls and velocity?
        //TODO and what about the two case in the unstuck same to the roller and the sandwich
        

        switch (transfer.getCurrentState().stateName) {
            case "IDLE":
                transfer.setVoltage(TransferConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                transfer.setVoltage(TransferConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                transfer.setVoltage(TransferConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION":
                transfer.setVoltage(TransferConstants.FEEDING_IN_MOTION_VOLTAGE);
                break;
            case "FEEDING_EJECT":
                transfer.setVoltage(TransferConstants.FEEDING_EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                transfer.setVoltage(TransferConstants.SHOOTING_VOLTAGE);
                break;
            case "UNSTUCK":
                transfer.setVoltage(TransferConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getR2()) {
            transfer.setVoltage(5); // TODO same as the others
        } else {
            transfer.setVoltage(0);
        }

    }

    @Override
    public void CantMove() {
        transfer.setVoltage(0);
    }
}
