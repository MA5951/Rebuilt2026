
package frc.robot.Subsystems.Shooter;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.VelocityControlledSystem;

public class Shooter extends VelocityControlledSystem {

    private static Shooter shooter;
    private Shooter() {
        super(ShooterConstants.shooterConstants);//TODO States

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