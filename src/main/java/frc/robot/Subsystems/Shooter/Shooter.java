
package frc.robot.Subsystems.Shooter;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.VelocityControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;

public class Shooter extends VelocityControlledSystem {

    private static Shooter shooter;
    private DigitalInput ir;

    private Shooter() {
        super(ShooterConstants.SHOOTER_CONSTANTS, ShooterConstants.IDLE, ShooterConstants.SHOOTING,
                ShooterConstants.WARMUP, ShooterConstants.FEEDING, ShooterConstants.FEEDING_IN_MOTION,
                ShooterConstants.EJECT);

        ir = new DigitalInput(PortMap.ShooterPorts.IR);
    }

    public boolean atPointForShooting() {
        return getError() < 100;
    }

    public boolean atPointForFeeding() {
        return getError() < 300;
    }

    public boolean atPointForFeedingInMotion() {
        return getError() < 300;
    }

    public boolean getSensor() {
        return ir.get();
    }

    @Override
    public void createSelfTest() {
        return;
    }

    @Override
    public boolean CAN_MOVE() {
        return true;
    }

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }

}