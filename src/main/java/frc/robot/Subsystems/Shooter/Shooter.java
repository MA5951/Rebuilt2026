
package frc.robot.Subsystems.Shooter;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.VelocityControlledSystem;

public class Shooter extends VelocityControlledSystem {

    private static Shooter shooter;

    //TODO need to add MACAM/IR

    private Shooter() {
        super(ShooterConstants.SHOOTER_CONSTANTS, ShooterConstants.IDLE, ShooterConstants.SHOOTING,
                ShooterConstants.WARMUP, ShooterConstants.FEEDING, ShooterConstants.FEEDING_IN_MOTION,
                ShooterConstants.EJECT);
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