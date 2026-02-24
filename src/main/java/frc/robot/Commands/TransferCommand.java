
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
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
        switch (transfer.getCurrentState().stateName) {
            case "IDLE":
                if ((!SixBar.getInstance().atPoint(15) && SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION)
                ||(SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION && SixBar.getInstance().getPosition() < -SixBarConstants.BUMPER_ZONE_ANGLE - 15)) {
                    transfer.setVoltage(3);
                } else {
                    transfer.setVoltage(TransferConstants.IDLE_VOLTAGE);
                }

                // transfer.setVoltage(TransferConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                transfer.setVoltage(TransferConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                transfer.setVoltage(TransferConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION":
                transfer.setVoltage(TransferConstants.SHOOTING_VOLTAGE);
                break;
            case "EJECT":
                transfer.setVoltage(TransferConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                transfer.setVoltage(SuperStructure.getTransferSinVoltage());
                //  if (!SuperStructure.isBallsInSandwich()) {
                //    transfer.setVoltage(5);
                // } else {
                //    transfer.setVoltage(0);
                // }
                //transfer.setVoltage(10);
                break;
            case "UNSTUCK":
                transfer.setVoltage(TransferConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getR2()) {

            transfer.setVoltage(TransferConstants.MANUAL_VOLTAGE);

        } else {
            transfer.setVoltage(0);
        }

    }

    @Override
    public void CantMove() {
        transfer.setVoltage(0);
    }
}
