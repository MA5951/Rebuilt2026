
package frc.robot.Commands;

import com.MAutils.DashBoard.DashBoard;
import com.MAutils.RobotControl.SubsystemCommand;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.Dashboard;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.IntakeRoller.IntakeRoller;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.Swerve;

public class IntakeCommand extends SubsystemCommand {
    private static IntakeRoller intakeroller = IntakeRoller.getInstance();

    public IntakeCommand() {
        super(intakeroller);
        addRequirements(intakeroller);
    }

    @Override
    public void Automatic() {
        switch (intakeroller.getCurrentState().stateName) {
            case "IDLE":
                if (((((!SixBar.getInstance().atPoint(15)
                        && SixBar.getInstance().getCurrentState() != SixBarConstants.COLLISION) ||
                        (SixBar.getInstance().getCurrentState() == SixBarConstants.COLLISION
                                && SixBar.getInstance().getPosition() < -SixBarConstants.BUMPER_ZONE_ANGLE - 15))))
                        && SixBar.getInstance().getCurrentState() != SixBar.HOMING) {
                    intakeroller.setVoltage(4);
                } else {
                    intakeroller.setVoltage(IntakeRollerConstants.IDLE_VOLTAGE);
                }

                // intakeroller.setVoltage(0);

                break;

            case "FORWARD":
                if (SixBar.getInstance().getPosition() < -60) {
                    if (DriverStation.isAutonomous()) {
                        intakeroller.setVoltage(10.5);
                    } else {
                        intakeroller.setVoltage(7);
                    }
                } else {
                    intakeroller.setVoltage(0);
                }
                break;
            case "BACKWARD":
                intakeroller.setVoltage(IntakeRollerConstants.BACKWARD_VOLTAGE);
                break;
            case "SHOOTING":
                // if (SixBar.getInstance().atPoint(3)) {
                // intakeroller.setVoltage(0);
                // } else {
                // intakeroller.setVoltage(4);
                // }
                if (SixBar.getInstance().atPoint(3) && SixBar.getInstance().getPosition() > -55) {
                    intakeroller.setVoltage(-6);
                } else {
                    intakeroller.setVoltage(6);
                }
                break;
        }
    }

    @Override
    public void Manual() {
        if (RobotContainer.getOperatorController().getR1()) {
            intakeroller.setVoltage(8); // TODO move to constants/use the saame value as automatic
        } else if (RobotContainer.getOperatorController().getL1()) {
            intakeroller.setVoltage(-8);
        } else {
            intakeroller.setVoltage(0);
        }
    }

    @Override
    public void CantMove() {
        intakeroller.setVoltage(0);
    }
}