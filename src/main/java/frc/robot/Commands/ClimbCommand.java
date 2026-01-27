
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

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

    }

    @Override
    public void CantMove() {
        climb.setVoltage(0);
    }

}
