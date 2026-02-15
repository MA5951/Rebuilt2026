
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class SixBarCommand extends SubsystemCommand {

    private static final SixBar sixbar = SixBar.getInstance();
    private double manuelPosition = SixBarConstants.IDLE_ANGLE;

    public SixBarCommand() {
        super(sixbar);
        addRequirements(sixbar);
    }

    @Override
    public void Automatic() {
        switch (sixbar.getCurrentState().stateName) {
            case "IDLE":
                if (SuperStructure.isDefenceMode()) {
                    sixbar.setPositionClose(SixBarConstants.FRAME_PARIMETER_ANGLE);
                } else {
                    sixbar.setPositionClose(SixBarConstants.BUMPER_ZONE_ANGLE);
                }
                break;
            case "DEPLOY":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
                break;
            case "ARMBRAKS":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
                break;
            case "COLLISION":
                if (sixbar.getPosition() > SixBarConstants.COLLISION_POWER_ANGLE) {
                    sixbar.setPositionClose(SixBarConstants.FRAME_PARIMETER_ANGLE);
                } else {
                    sixbar.setVoltage(SixBarConstants.COLLISION_VOLTS_OUTSIDE);
                }

               
                break;
            case "SHOOTING":
                sixbar.setPosition(SixBarConstants.SHOOTING_ANGLE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getActionsUp()) {
            manuelPosition = SixBarConstants.DEPLOY_ANGLE;
            // sixbar.setVoltage(1);
        } else if (RobotContainer.getOperatorController().getActionsDown()) {
            manuelPosition = SixBarConstants.IDLE_ANGLE;
            // sixbar.setVoltage(-1);
        } 
        sixbar.setPosition(manuelPosition);
    }

    @Override
    public void CantMove() {
        sixbar.setVoltage(SixBarConstants.CANT_MOVE_VOLTAGE);
    }

}
