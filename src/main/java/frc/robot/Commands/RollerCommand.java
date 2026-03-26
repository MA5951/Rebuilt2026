
package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Roller.RollerConstants;

public class RollerCommand extends SubsystemCommand {
    private static final Roller roller = Roller.getInstance();
    private double endTime = 0;

    public RollerCommand() {
        super(roller);
        addRequirements(roller);
    }

    @Override
    public void Automatic() {
        switch (roller.getCurrentState().stateName) {
            case "IDLE":
                roller.setVoltage(RollerConstants.IDLE_VOLTAGE);
                break;
            case "INTAKE":
                roller.setVoltage(RollerConstants.INTAKE_VOLTAGE);
                break;
            case "FEEDING":
                roller.setVoltage(RollerConstants.SHOOTING_VOLTAGE);
                break;
            case "FEDDING_IN_MOTION":
                roller.setVoltage(RollerConstants.SHOOTING_VOLTAGE);
                break;
            case "EJECT":
                roller.setVoltage(RollerConstants.EJECT_VOLTAGE);
                break;
            case "SHOOTING":
                //if (!SuperStructure.isBallsInSandwich()) {
                    //roller.setVoltage(6);
                // } else {
                //    roller.setVoltage(0);
                // }

                // if(Timer.getFPGATimestamp() - SuperStructure.startShootingTime > 1) {
                //     if (Roller.getInstance().getCurrent() > 25 || Timer.getFPGATimestamp() - endTime < 0.3) {
                //         endTime = Timer.getFPGATimestamp();
                //         roller.setVoltage(0);
                //     } else {
                //         roller.setVoltage(6);
                //     }
                //   } else {
                    roller.setVoltage(6);
                //   }

                // if (!SuperStructure.isBallsInSandwich()) {
                    // roller.setVoltage(6);
                   // roller.setVoltage(2);

                // } else {
                //   roller.setVoltage(0);
                // }
                break;
            case "UNSTUCK":
                roller.setVoltage(RollerConstants.UNSTUCK_VOLTAGE);
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getL2()) {
            roller.setVoltage(6.5); //TODO move to constants
        } else {
            roller.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        roller.setVoltage(0);
    }

}
