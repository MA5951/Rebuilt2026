
package frc.robot.Subsystems.Roller;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class Roller extends PowerControlledSystem {
    private static Roller roller;

    private Roller() {
        super(RollerConstants.ROLLER_CONSTANTS, RollerConstants.IDLE, RollerConstants.INTAKE,
                RollerConstants.FEEDING, RollerConstants.FEEDING_IN_MOTION,
                RollerConstants.EJECT, RollerConstants.SHOOTING,
                RollerConstants.UNSTUCK);
    }

    @Override
    public void createSelfTest() {

    }

    private boolean canShoot() {
                return (SuperStructure.atPointForShooting()) &&
                                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                                                || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS);
        }

    private boolean canFeedingInMotion() {
        return (SuperStructure.atPointForFeedingInMotion()
                && RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION);
    }

    private boolean canFeeding() {
        return (SuperStructure.atPointForFeeding() && RobotContainer.getRobotState() == RobotConstants.FEEDING);
    }

    private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY ||
                RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallsInSandwich();
    }

    @Override
    public boolean CAN_MOVE() {
        return canShoot() || canIntake() || canFeedingInMotion() || canFeeding()
                || RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                || RobotContainer.getRobotState() == RobotConstants.EJECT
                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                || RobotContainer.getRobotState() == RobotConstants.IDLE;

    }

    public static Roller getInstance() {
        if (roller == null) {
            roller = new Roller();
        }
        return roller;
    }

}
