
package frc.robot.Subsystems.Sandwich;

import com.MAutils.Components.MACam;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class Sandwich extends PowerControlledSystem {
    private static Sandwich sandwich;

    private double lastMAcamDistance;

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
        return (Swerve.getInstance().atPointForShooting() && Hood.getInstance().atPoint()
                && Shooter.getInstance().atPoint()) &&
                (RobotContainer.getRobotState() == RobotConstants.SHOOTING ||
                        RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                        || RobotContainer.getRobotState() == RobotConstants.FEEDING);// TODO change to larger tolerance
    }

    private boolean canIntake() {
        return (RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY ||
                RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER) && !SuperStructure.isBallsInSandwich();
    }

    private boolean canFeedingInMotion() { // change to larger tolerance
        return (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION && Hood.getInstance().atPoint()
                && Shooter.getInstance().atPoint() && !SuperStructure.isHittingNet() && !SuperStructure.outSideField());
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

    public double getMACamDistance() {
        return macam.getDistance();
    }

    public boolean getIRSensor() {
        return ir.get();
    }

    public double getDeltaMAcamDistance() {
        return macam.getDistance() - lastMAcamDistance;
    }

    public static Sandwich getInstance() {
        if (sandwich == null) {
            sandwich = new Sandwich();
        }
        return sandwich;
    }
}