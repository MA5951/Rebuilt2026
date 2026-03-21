
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.SixBar.SixBar;

public class ClimbCommand extends SubsystemCommand {
    private static final Climb climb = Climb.getInstance();
    private static Debouncer debouncer = new Debouncer(0.2);
    public static boolean isAtPosition = false;
    private static boolean climbLatch = false;
    private double lastCurrent = 0;

    public ClimbCommand() {
        super(climb);
        addRequirements(climb);
    }

    @Override
    public void Automatic() {
        switch (climb.getCurrentState().stateName) {
            case "IDLE":
                climb.setPosition(ClimbConstnats.IDLE_POSITION);
                break;
            case "PRECLIMB":
            
                if (isAtPosition) {
                    climb.setVoltage(0);
                } else {
                    climb.setPosition(ClimbConstnats.OPEN_POSITION,0.28);
                }

                if (climb.atPoint()) {
                    isAtPosition = true;
                }
                climbLatch = false;
                break;
            case "CLIMB":
                if (climb.getPosition() > ClimbConstnats.CLOSE_POSITION + 0.02 && !climbLatch) {
                    climb.setVoltage(ClimbConstnats.START_CLOSE_VOLTAGE);
                } else {
                    climbLatch = true;
                    climb.setVoltage(ClimbConstnats.END_CLOSE_VOLTAGE);
                }

              break;
            case "DOWN":
                climb.setPosition(ClimbConstnats.OPEN_POSITION);
                break;
            case "HOMING":
                climb.setVoltage(ClimbConstnats.HOMING_VOLTAGE);
                if (climb.getCurrent()-lastCurrent > HoodConstants.DELTA_CURRENT && climb.getCurrent() < 45) {
                    climb.resetPosition(0);
                    climb.setState(HoodConstants.IDLE);
                }
                lastCurrent = climb.getCurrent();
                break;
            case "SHOOTING":
                if(SuperStructure.isExtendedMagazine()){
                    if (climb.getPosition() > ClimbConstnats.CLOSE_POSITION + 0.01 ) {
                        climb.setVoltage(-0.8);
                    } else {
                        climb.setVoltage(-0.3);
                    }
                } else {
                    climb.setPosition(ClimbConstnats.IDLE_POSITION);
                }
                // } else {
                // if(SuperStructure.isExtendedMagazine()) {
                //     if (SixBarCommand.hasClosed) {
                //         climb.setPosition(ClimbConstnats.IDLE_POSITION);
                //     } else {
                //         climb.setVoltage(0);
                //     }
                // } else {
                //     climb.setPosition(ClimbConstnats.IDLE_POSITION);
                // }
                // climb.setPosition(ClimbConstnats.OPEN_POSITION);
                
                break;
            case "EXTEND":
                if(SuperStructure.isExtendedMagazine()){
                    climb.setPosition(0.089,0.19);
                } else {
                    climb.setPosition(ClimbConstnats.IDLE_POSITION);
                }
                break;
            case "HOLD":
            if(SuperStructure.isExtendedMagazine()){
                    climb.setPosition(0.089,0.19);
                } else {
                    climb.setPosition(ClimbConstnats.IDLE_POSITION);
                }
                break;

        }
    }

    @Override
    public void Manual() {
        climb.setVoltage(-RobotContainer.getOperatorController().getLeftY(true, 3));
    }

    @Override
    public void CantMove() {
        climb.setVoltage(0);
    }

}
