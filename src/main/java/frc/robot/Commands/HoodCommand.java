

package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;


public class HoodCommand extends SubsystemCommand {
    private static final Hood hood = Hood.getInstance();

    private double manuelPosition = 20;

    public HoodCommand() {
        super(hood);
        addRequirements(hood);
    }

    @Override
    public void Automatic() {
        switch (hood.getCurrentState().stateName) {
            case "IDLE":
                hood.setPosition(HoodConstants.IDLE_POSITION);
                break;
            case "SHOOTING":
                hood.setPosition(SuperStructure.getShootingParameters().hoodAngle());
                break;
            case "FEEDING":
                hood.setPosition(SuperStructure.getFeedingParameters().hoodAngle());
                break;
            case "FEEDING_IN_MOTION":
                hood.setPosition(SuperStructure.getFeedingParameters().hoodAngle());
                break;
            case "EJECT":
                hood.setPosition(HoodConstants.EJECT_POSITION);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getDpadUp()) {
            manuelPosition += 2;
        }   else if (RobotContainer.getOperatorController().getDpadDown()) {
            manuelPosition -= 2;
        }
        hood.setPosition(manuelPosition);
    }

    @Override
    public void CantMove() {
        hood.setVoltage(0);
    }
}
