
package frc.robot.Subsystems.IntakeRoller;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class IntakeRoller extends PowerControlledSystem {
    private static IntakeRoller intakeroller;

    private IntakeRoller() {
        super(IntakeRollerConstants.INTAKE_ROLLER_CONSTANTS, IntakeRollerConstants.IDLE,
                IntakeRollerConstants.FORWARD, IntakeRollerConstants.BACKWARD, IntakeRollerConstants.SHOOTING);

    }

    @Override
    public boolean CAN_MOVE() {
        return !SuperStructure.isFull() && (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY
                || RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER)
                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                || RobotContainer.getRobotState() == RobotConstants.IDLE
                || RobotContainer.getRobotState() == RobotConstants.SHOOTING
                || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION;
    }

    @Override
    public void createSelfTest() {

    }

    public static IntakeRoller getInstance() {
        if (intakeroller == null) {
            intakeroller = new IntakeRoller();
        }
        return intakeroller;
    }

}
