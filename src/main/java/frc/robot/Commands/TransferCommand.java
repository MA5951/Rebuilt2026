
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class TransferCommand extends SubsystemCommand {
    private static Transfer transfer = Transfer.getInstance();
    private double endTime = 0;

    public TransferCommand() {
        super(transfer);
        addRequirements(transfer);
    }

    @Override
    public void Automatic() {
        switch (transfer.getCurrentState().stateName) {
            case "IDLE":
                if ((!SixBar.getInstance().atPoint(15) &&
                        SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION)
                        || (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION &&
                                SixBar.getInstance().getPosition() < -SixBarConstants.BUMPER_ZONE_ANGLE -
                                        15)) {
                    transfer.setVoltage(3);
                } else {
                    transfer.setVoltage(TransferConstants.IDLE_VOLTAGE);
                }

                break;
            case "INTAKE":
                transfer.setVoltage(TransferConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                transfer.setVoltage(TransferConstants.FEEDING_VOLTAGE);
                break;
            case "FEEDING_IN_MOTION":
                transfer.setVoltage(SuperStructure.getTransferSinVoltage());
                break;
            case "EJECT":
                transfer.setVoltage(TransferConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":

                transfer.setVoltage(SuperStructure.getTransferSinVoltage());

                // if(Timer.getFPGATimestamp() - SuperStructure.startShootingTime < 3) {
                // transfer.setVoltage(4 + Timer.getFPGATimestamp() -
                // SuperStructure.startShootingTime);
                // } else {
                // transfer.setVoltage(7);
                // }

                // if(Timer.getFPGATimestamp() - SuperStructure.startShootingTime > 1) {
                // if (Roller.getInstance().getCurrent() > 25 || Timer.getFPGATimestamp() -
                // endTime < 0.3) {
                // endTime = Timer.getFPGATimestamp();
                // transfer.setVoltage(0);
                // } else {
                // transfer.setVoltage(7);
                // }\
                // } else {
                // transfer.setVoltage(7);
                // }

                // transfer.setVoltage(7);

                // if (!SuperStructure.isBallsInSandwich()) {
                // transfer.setVoltage(8);
                // } else {
                // transfer.setVoltage(3);
                // }
                // transfer.setVoltage(4);
                // why no work? GALDO
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
