
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;

public class SixBarCommand extends SubsystemCommand {

    private static final SixBar sixbar = SixBar.getInstance();
    private double manuelPosition = SixBarConstants.IDLE_ANGLE;
    private static Debouncer homingDebouncer = new Debouncer(0.1);
    public static boolean isAtPosition = false;
    public static boolean isReset = false;
    public double lastCurrent = 0;

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

                // if (SixBar.getInstance().getPosition() < -36) {
                //     sixbar.setPosition(34.5);
                // } else {
                //     sixbar.setPosition(8);
                // }

                if (SixBar.getInstance().getPosition() < -20) {
                    sixbar.setPosition(SixBarConstants.FRAME_PARIMETER_ANGLE);
                } else if (SixBar.getInstance().getPosition() > -5){
                    sixbar.setPosition(SixBarConstants.BUMPER_ZONE_ANGLE);
                } 

                // sixbar.setPosition(SixBarConstants.FRAME_PARIMETER_ANGLE);
                // sixbar.setVoltage(1.5);
                break;
            case "HOMING":
                isReset = true;
                sixbar.setVoltage(1.2);
                if (homingDebouncer.calculate(sixbar.getCurrent() > 50)) {
                    sixbar.resetPosition(0);
                    sixbar.setState(SixBarConstants.IDLE);
                }
                lastCurrent = sixbar.getCurrent();
                break;
            case "FORCE_OPEN":
                sixbar.setVoltage(-1.3);
                break;
            case "FORCE_CLOSE":
                sixbar.setVoltage(1.3);
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
