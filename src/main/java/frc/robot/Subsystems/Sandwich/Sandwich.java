
package frc.robot.Subsystems.Sandwich;

import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class Sandwich extends PowerControlledSystem {
    private static Sandwich sandwich;


    private DigitalInput leftIr;
    private DigitalInput rightIr;
    private DigitalInput middleIr;

    private Sandwich() {
        super(SandwichConstants.SANDWICH_CONSTANTS, SandwichConstants.IDLE, SandwichConstants.INTAKE,
                SandwichConstants.FEEDING,
                SandwichConstants.FEEDING_IN_MOTION, SandwichConstants.EJECT,
                SandwichConstants.SHOOTING, SandwichConstants.UNSTUCK);

        leftIr = new DigitalInput(PortMap.Sandwich_Ports.LEFT_IR);
        rightIr = new DigitalInput(PortMap.Sandwich_Ports.RIGHT_IR);
        middleIr = new DigitalInput(PortMap.Sandwich_Ports.MIDDLE_IR);
    }

    @Override
    public void createSelfTest() {
    }

    private boolean canShoot() {
        return (SuperStructure.atPointForShooting()) &&
                ((RobotContainer.getRobotState() == RobotConstants.SHOOTING
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS)) ;
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
                || RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                || RobotContainer.getRobotState() == RobotConstants.EJECT
                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                || RobotContainer.getRobotState() == RobotConstants.IDLE;

    }

    public boolean getLeftIr() {
        return !leftIr.get();
    }

    public boolean getRightIr() {
        return !rightIr.get();
    }

    public boolean getMiddleIr() {
        return !middleIr.get();
    }

    

    @Override
    public void periodic() {
        super.periodic();

        MALog.log("/Subsystems/Sandwich/ left ir", getLeftIr());
        MALog.log("/Subsystems/Sandwich/ right ir", getRightIr());
        MALog.log("/Subsystems/Sandwich/ middle ir", getMiddleIr());


    }

    public static Sandwich getInstance() {
        if (sandwich == null) {
            sandwich = new Sandwich();
        }
        return sandwich;
    }
}