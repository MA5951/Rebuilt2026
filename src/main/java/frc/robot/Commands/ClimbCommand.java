
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;

public class ClimbCommand extends SubsystemCommand {
    private static final Climb climb = Climb.getInstance();

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
                climb.setPosition(ClimbConstnats.OPEN_POSITION);
                break;
            case "CLIMB":
                climb.setPosition(ClimbConstnats.CLOSE_POSITION);
                break;
            case "DOWN":
                climb.setPosition(ClimbConstnats.CLOSE_POSITION);
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
