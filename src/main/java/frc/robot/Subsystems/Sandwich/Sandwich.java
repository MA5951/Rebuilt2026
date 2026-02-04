
package frc.robot.Subsystems.Sandwich;

import com.MAutils.Components.MACam;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class Sandwich extends PowerControlledSystem {
    private static Sandwich sandwich;

    private double lastMAcamDistance;
    private double deltaMAcamDistance;

    private MACam macam = new MACam(PortMap.Sandwich_Ports.MACAM);
    private DigitalInput ir = new DigitalInput(PortMap.Sandwich_Ports.IR);

    private Sandwich() {
        super(SandwichConstants.SANDWICH_CONSTANTS, SandwichConstants.IDLE, SandwichConstants.INTAKE,
                SandwichConstants.FEEDING,
                SandwichConstants.FEEDING_IN_MOTION, SandwichConstants.EJECT,
                SandwichConstants.SHOOTING, SandwichConstants.UNSTUCK);

        macam = new MACam(PortMap.Sandwich_Ports.MACAM);
        ir = new DigitalInput(PortMap.Sandwich_Ports.IR);

        lastMAcamDistance = macam.getDistance();

    }

    @Override
    public void createSelfTest() {
    }

    private boolean canShoot() {
        return (SuperStructure.atPointForShooting()) &&
                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS);
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

    public double getMACamDistance() {
        return macam.getDistance();
    }

    public boolean getEndSensor() {
        return ir.get();
    }

    public double getDeltaMAcamDistance() {
        deltaMAcamDistance = macam.getDistance() - lastMAcamDistance;
        lastMAcamDistance = macam.getDistance();
        return deltaMAcamDistance;
    }

    @Override
    public void periodic() {
        super.periodic();
        MALog.log("Subsystems/Sandwich/MACam Distance", getMACamDistance()); 
        MALog.log("Subsystems/Sandwich/End Sensor", getEndSensor());   

    }

    public static Sandwich getInstance() {
        if (sandwich == null) {
            sandwich = new Sandwich();
        }
        return sandwich;
    }
}