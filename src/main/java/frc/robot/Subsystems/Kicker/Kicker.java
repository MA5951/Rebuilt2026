
package frc.robot.Subsystems.Kicker;


import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;


import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class Kicker extends PowerControlledSystem {
    private static Kicker kicker;

    private Kicker() {
        super(KickerConstants.KICKER_CONSTANTS, KickerConstants.IDLE, KickerConstants.INTAKE,
                KickerConstants.FEEDING,
                KickerConstants.FEEDING_IN_MOTION, KickerConstants.EJECT,
                KickerConstants.SHOOTING);

    }

    @Override
    public void createSelfTest() {
    }

    private boolean canShoot() {
        return (SuperStructure.atPointForShooting()) &&
                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS) ;
    }

    private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY ||
                RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallsInSandwich();
    }

    private boolean canFeedingInMotion() {
        return (SuperStructure.atPointForFeedingInMotion()
                && RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION);
    }

    private boolean canFeeding() {
        return (SuperStructure.atPointForFeeding() && RobotContainer.getRobotState() == RobotConstants.FEEDING);
    }

    @Override
    public boolean CAN_MOVE() {
        return canShoot() || canIntake() || canFeedingInMotion() || canFeeding()
                || RobotContainer.getRobotState() == RobotConstants.EJECT
                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                || RobotContainer.getRobotState() == RobotConstants.IDLE;

    }

    public static Kicker getInstance() {
        if (kicker == null) {
            kicker = new Kicker();
        }
        return kicker;
    }
}