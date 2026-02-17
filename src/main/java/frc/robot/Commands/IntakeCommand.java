
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

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
                // if ((!SixBar.getInstance().atPoint(15) && SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION) || 
                // (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION && SixBar.getInstance().getPosition() < -SixBarConstants.BUMPER_ZONE_ANGLE - 15)) {
                //     intakeroller.setVoltage(7);
                // } else {
                //     intakeroller.setVoltage(IntakeRollerConstants.IDLE_VOLTAGE);
                // }

                intakeroller.setVoltage(IntakeRollerConstants.IDLE_VOLTAGE);
                break;

            case "FORWARD":
                intakeroller.setVoltage(7);//9
                break;
            case "BACKWARD":
                intakeroller.setVoltage(IntakeRollerConstants.BACKWARD_VOLTAGE);
                break;
            case "SHOOTING":
                if ((!SixBar.getInstance().atPoint(15) && SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION) || 
                (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION && SixBar.getInstance().getPosition() < -SixBarConstants.BUMPER_ZONE_ANGLE - 15)) {
                    intakeroller.setVoltage(11);
                } else {
                    intakeroller.setVoltage(3);
                }
            break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getR1()) {
            intakeroller.setVoltage(8); //TODO move to constants/use the saame value as automatic
        } else if (RobotContainer.getOperatorController().getL1()){
            intakeroller.setVoltage(-8);
        } else {
            intakeroller.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        intakeroller.setVoltage(0);
    }
}