
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class SixBarCommand extends SubsystemCommand {

    private static final SixBar sixbar = SixBar.getInstance();
    private double manuelPosition = SixBarConstants.FRAME_PARIMETER_ANGLE;

    public SixBarCommand() {
        super(sixbar);
        addRequirements(sixbar);
    }

    @Override
    public void Automatic() {
        switch (sixbar.getCurrentState().stateName) {
            case "IDLE":
                if (SuperStructure.isDefenceMode()) {
                    sixbar.setPosition(SixBarConstants.FRAME_PARIMETER_ANGLE);
                } else {
                    sixbar.setPosition(SixBarConstants.BUMPER_ZONE_ANGLE);
                }
                break;
            case "DEPLOY":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
            case "ARMBRAKS":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
                break;
            case "COLLISION":
                sixbar.setVoltage(SixBarConstants.COLLISION_VOLTS);
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
        } else if (RobotContainer.getOperatorController().getActionsDown()) {
            manuelPosition = SixBarConstants.FRAME_PARIMETER_ANGLE;
        }
        sixbar.setPosition(manuelPosition);
    }

    @Override
    public void CantMove() {
        sixbar.setVoltage(0);
    }

}
