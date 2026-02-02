
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
                //TODO add reset position
                break;
            case "DEPLOY":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
            case "ARMBRAKS":
                sixbar.setPosition(SixBarConstants.DEPLOY_ANGLE);
                //TODO after its in postion apply a small voltage to hold it there instad of position control, it for the collision state
                break;
            case "COLLISION":
                sixbar.setVoltage(SixBarConstants.COLLISION_VOLTS);
                //TODO why dont implement as we takled in the subsystem design doc? give it a smalell voltage where it go over the pivot point
                break;
            case "SHOOTING":
                sixbar.setPosition(SixBarConstants.SHOOTING_ANGLE);
                //probably the intake should open and close so write it as an else if in a big tolrance
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
        //TODO what about the go to  COLLISION if the current are too high?
        sixbar.setVoltage(0);
    }

}
