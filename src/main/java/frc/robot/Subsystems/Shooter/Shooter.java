
package frc.robot.Subsystems.Shooter;

import com.MAutils.DashBoard.Tunable;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.VelocityControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;

public class Shooter extends VelocityControlledSystem {

    private static Shooter shooter;
    private DigitalInput ir;
    private Tunable shooterVelo;

    private Shooter() {
        super(ShooterConstants.SHOOTER_CONSTANTS, ShooterConstants.IDLE, ShooterConstants.SHOOTING,
                ShooterConstants.WARMUP, ShooterConstants.FEEDING, ShooterConstants.FEEDING_IN_MOTION,
                ShooterConstants.EJECT);

        ir = new DigitalInput(PortMap.ShooterPorts.IR);
        shooterVelo = new Tunable(0, "Shooter Velocity");
    }

    public boolean atPointForShooting() {
        return Math.abs(getError()) < ShooterConstants.AT_POINT_FOR_SHOOTING_TOLERANCE;
    }

    public boolean atPointForFeeding() {
        return Math.abs(getError()) < ShooterConstants.AT_POINT_FOR_FEEDING_TOLERANCE;
    }

    public boolean atPointForFeedingInMotion() {
        return Math.abs(getError()) < ShooterConstants.AT_POINT_FOR_FEEDING_TOLERANCE;
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

    public double getShooterVelo() {
        return shooterVelo.get();
    }

    @Override
    public void periodic() {
        super.periodic();
        MALog.log("Subsystems/Shooter/is balls", getSensor());
    }

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }

}