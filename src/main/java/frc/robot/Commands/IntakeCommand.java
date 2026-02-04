
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;

public class IntakeCommand extends SubsystemCommand {
    private static IntakeRoller intakeroller = IntakeRoller.getInstance();

    public IntakeCommand() {
        super(intakeroller);
        addRequirements(intakeroller);
    }

    @Override
    public void Automatic() {
        switch (intakeroller.getCurrentState().stateName) {
            case "IDLE":
                intakeroller.setVoltage(IntakeRollerConstants.IDLE_VOLTAGE);
                break;
            case "FORWARD":
                intakeroller.setVoltage(IntakeRollerConstants.FORWARD_VOLTAGE);
                break;
            case "BACKWARD":
                intakeroller.setVoltage(IntakeRollerConstants.BACKWARD_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getR1()) {
            intakeroller.setVoltage(6); //TODO move to constants/use the saame value as automatic
        } else if (RobotContainer.getOperatorController().getL1()){
            intakeroller.setVoltage(-6);
        } else {
            intakeroller.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        intakeroller.setVoltage(0);
    }
}