
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class SixBarCommand extends SubsystemCommand {

    private static final SixBar sixbar = SixBar.getInstance();

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
                sixbar.setVoltage(SixBarConstants.COLLISION_VOLTS); //TODO change to zero and if the 6bar angle is leed the x give it -1 volt
                break;
            case "SHOOTING":
                sixbar.setPosition(SixBarConstants.SHOOTING_ANGLE);
                break;
        }
    }

    @Override
    public void Manual() {
        //TODO add

    }

    @Override
    public void CantMove() {
        sixbar.setVoltage(0);
    }

}
