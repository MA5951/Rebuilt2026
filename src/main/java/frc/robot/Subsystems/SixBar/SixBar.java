
package frc.robot.Subsystems.SixBar;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import frc.robot.PortMap;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Swerve.Swerve;

public class SixBar extends PositionControlledSystem {

    private static SixBar sixbar;

    private StatusSignal<Double> closedLoopVolts;
    private double lastVelo = 0;

    public static final State HOMING = new State("HOMING");

    private SixBar() {
        super(SixBarConstants.SIXBAR_CONSTANTS, SixBarConstants.ARMBRAKS, SixBarConstants.COLLISION,
                SixBarConstants.SHOOTING, SixBarConstants.IDLE, SixBarConstants.DEPLOY, HOMING);

        closedLoopVolts = systemIO.getSystemConstants().master.motorController.getClosedLoopOutput();
        


        resetPosition(-SixBarConstants.DEPLOY_ANGLE);

        HOMING.setOnStateSet(() -> setConstants(SixBarConstants.HOMING_SIXBAR_CONSTANTS, false));
        HOMING.setOnStateEnd(() -> setConstants(SixBarConstants.SIXBAR_CONSTANTS, false));

        StatusSignalsRunner.registerSignals(PortMap.SixBarPorts.SIXBAR_MOTOR, closedLoopVolts);
    }

    public double getCloseLoopVolts() {
        return closedLoopVolts.getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(-position, 0.3  - (Math.abs(Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond) * 0.22));
    }

    public void setPositionClose(double position) {
        super.setPosition(-position, -0.2 + (Math.abs(Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond)));
    }

    @Override
    public void createSelfTest() {

    }

    public boolean atPoint(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    @Override
    public boolean CAN_MOVE() {
        return Climb.getInstance().getPosition() < ClimbConstnats.CLIMB_INTAKE_POSE;
    }


    @Override
    public void periodic() {
        super.periodic();
        MALog.log(LOG_PATH + "PID Voltage", getCloseLoopVolts());
        MALog.log(LOG_PATH + "Acceleratin", lastVelo - getVelocity());
        MALog.log(LOG_PATH + "HommingCondition", Math.abs(sixbar.getCurrent()) > 35);

        lastVelo = getVelocity();

    }

    public static SixBar getInstance() {
        if (sixbar == null) {
            sixbar = new SixBar();
        }
        return sixbar;
    }
}
