
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class ShooterCommand extends SubsystemCommand {
    private static final Shooter shooter = Shooter.getInstance();

    public ShooterCommand() {
        super(shooter);
    }

    @Override
    public void Automatic() {
        switch (shooter.getCurrentState().stateName) {
            case "IDLE":
                shooter.setVelocity(ShooterConstants.IDLE_VELOCITY);
                break;
            case "WARMUP":
                shooter.setVelocity(ShooterConstants.WARMUP_VELOCITY);
                break;
            case "SHOOTING":
                shooter.setVelocity(Superstructure.getShooterVelocity());// TODO after merge
                break;
            case "FEEDING":
                shooter.setVelocity(Superstructure.getFeedingVelocity());// TODO after merge
                break;
            case "STATIC_FEEDING":
                shooter.setVelocity(Superstructure.getStaticFeedingVelocity());// TODO after merge
                break;
            case "EJECT":
                shooter.setVelocity(ShooterConstants.EJECT_VELOCITY);
                break;

        }
    }

    @Override
    public void Manual() {
    }

    @Override
    public void CantMove() {
        shooter.setVoltage(0);
    }
}
