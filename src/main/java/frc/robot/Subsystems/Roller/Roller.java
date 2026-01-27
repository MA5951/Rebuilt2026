
package frc.robot.Subsystems.Roller;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter.Shooter;

public class Roller extends PowerControlledSystem {
    private static Roller roller;

    private Roller() {
        super(RollerConstants.ROLLER_CONSTANTS, RollerConstants.IDLE, RollerConstants.INTAKE,
                RollerConstants.FEEDING, RollerConstants.FEDDING_IN_MOTION,
                RollerConstants.EJECT, RollerConstants.SHOOTING,
                RollerConstants.UNSTUCK);
    }

    @Override
    public void createSelfTest() {
        
    }

    private boolean canShoot() {
                return (Swerve.atPointForSHooting() && hood.atPointForSHooting() && Shooter.atPointForSHooting()) &&
                        (RobotContainer.getRobotState() == RobotConstants.SHOOTING 
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                        || RobotContainer.getRobotState() == RobotConstants.FEEDING);// TODO After Merge
        }

        private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY ||
                RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallDetected();
    }

        private boolean canFeedingInMotion() {
                return (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION && hood.atPointForFeeding()
                                && Shooter.atPointForFeeding && !SuperStructure.isHittingNet()
                                && !SuperStructur.outSideField());// TODO After Merge
        }

        @Override
        public boolean CAN_MOVE() {
                return canShoot() || canIntake() || canFeedingInMotion()
                                || RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                                || RobotContainer.getRobotState() == RobotConstants.EJECT
                                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                                || RobotContainer.getRobotState() == RobotConstants.IDLE; // TODO After Merge

        }

    public static Roller getInstance() {
        if (roller == null) {
            roller = new Roller();
        }
        return roller;
    }

}
