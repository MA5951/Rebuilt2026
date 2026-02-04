
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
                climb.unlockClimb();
                climb.setPosition(ClimbConstnats.IDLE_POSITION);
                break;
            case "PRECLIMB":
                climb.setPosition(ClimbConstnats.OPEN_POSITION);
                break;
            case "CLIMB":
                if (climb.getPosition() > ClimbConstnats.LOCK_POSITION) {
                    climb.setPosition(ClimbConstnats.CLOSE_POSITION);
                } else {
                    climb.lockClimb();
                    climb.setVoltage(ClimbConstnats.LOCK_VOLTAGE);
                }
                break;
            case "DOWN":
                climb.unlockClimb();
                climb.setPosition(ClimbConstnats.OPEN_POSITION);
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

        if (climb.getCurrentState() == ClimbConstnats.DOWN) {
            climb.unlockClimb();
        }
    }

}
