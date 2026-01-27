
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
                climb.setVoltage(0);
                break;
            case "PRECLIMB":
                climb.setPosition(ClimbConstnats.PRECLIMB_POSITION);
                break;
            case "CLIMB":
                climb.setPosition(ClimbConstnats.CLIMB_POSITION);
                break;
            case "DOWN":
                climb.setPosition(ClimbConstnats.START_POSITION);
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
