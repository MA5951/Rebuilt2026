
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.Swerve.Swerve;

public class HoodCommand extends SubsystemCommand {
    private static final Hood hood = Hood.getInstance();
    private static Debouncer homingDebouncer = new Debouncer(0.2);
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
            case "HOMING":
                hood.setVoltage(HoodConstants.HOMING_VOLTAGE);
                if (homingDebouncer.calculate(Math.abs(hood.getCurrent()) > HoodConstants.HOMING_CURRENT)) {
                    hood.resetPosition(0);
                    hood.setState(HoodConstants.IDLE);
                }
                break;
        }
    }

    @Override
    public void Manual() {
        // if (manuelPosition < HoodConstants.MIN_POSITION) {
        // manuelPosition = HoodConstants.MIN_POSITION;
        // } else if (manuelPosition > HoodConstants.MAX_POSITION) {
        // manuelPosition = HoodConstants.MAX_POSITION;
        // }

        // if (RobotContainer.getOperatorController().getDpadUp()) {
        // //manuelPosition += HoodConstants.MANUAL_INCREMENT;
        // hood.setVoltage(1);
        // } else if (RobotContainer.getOperatorController().getDpadDown()) {
        // //manuelPosition -= HoodConstants.MANUAL_INCREMENT;
        // hood.setVoltage(-1);
        // } else {
        // hood.setVoltage(0);
        // }

        // if (RobotContainer.getOperatorController().getActionsLeft()) {
        // manuelPosition = 23;
        // } else if (RobotContainer.getOperatorController().getActionsRight()) {
        // manuelPosition = 5;
        // }
        hood.setPosition(hood.getHoodPosition());
    }

    @Override
    public void CantMove() {
        hood.setVoltage(0);
    }
}
