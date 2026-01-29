
package frc.robot.Subsystems.Roller;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.Shooter;
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
        return (Swerve.getInstance().atPointForShooting() && Hood.getInstance().atPoint()
                && Shooter.getInstance().atPoint()) &&
                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                        || RobotContainer.getRobotState() == RobotConstants.FEEDING);// TODO change to larger tolerance
    }

    private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY ||
                RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallsInSandwich();
    }

    private boolean canFeedingInMotion() {
        return (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION && Hood.getInstance().atPoint()
                && Shooter.getInstance().atPoint() && !SuperStructure.isHittingNet()
                && !SuperStructure.outSideField());// change to larger tolerance
    }

    @Override
    public boolean CAN_MOVE() {
        return canShoot() || canIntake() || canFeedingInMotion()
                || RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                || RobotContainer.getRobotState() == RobotConstants.EJECT
                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                || RobotContainer.getRobotState() == RobotConstants.IDLE;

    }

    public boolean isMoving() {
        return getVelocity() > RollerConstants.MAX_VELOCITY_IN_STOPING;
    }

    public static Roller getInstance() {
        if (roller == null) {
            roller = new Roller();
        }
        return roller;
    }

}
