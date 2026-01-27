
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;

public class IntakeCommand extends SubsystemCommand{
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
        
    }

    @Override
    public void CantMove() {
        intakeroller.setVoltage(0);
    }
}