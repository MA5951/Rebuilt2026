
package frc.robot.Subsystems.SixBar;


import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import frc.robot.PortMap;

public class SixBar extends PositionControlledSystem {

    private static SixBar sixbar;

    private StatusSignal<Double> closedLoopVolts;
    private StatusSignal<Angle> absPosition;
    private final CANcoder canCoder;

    private SixBar() {
        super(SixBarConstants.SIXBAR_CONSTANTS, SixBarConstants.ARMBRAKS, SixBarConstants.COLLISION,
                SixBarConstants.SHOOTING, SixBarConstants.IDLE, SixBarConstants.DEPLOY);

        closedLoopVolts = systemIO.getSystemConstants().master.motorController.getClosedLoopOutput();
        canCoder = new CANcoder(PortMap.SixBarPorts.CAN_CODER, PortMap.CAN_BUS.CANIVORE_BUS);

        absPosition = canCoder.getAbsolutePosition();
        absPosition.refresh();

        resetPosition((absPosition.getValueAsDouble() * 360) / SixBarConstants.CAN_CODER_GEAR);

        StatusSignalsRunner.registerSignals(PortMap.SixBarPorts.SIXBAR_MOTOR, closedLoopVolts);
        StatusSignalsRunner.registerSignals(true,  absPosition);
    }

    public double getCloseLoopVolts() {
        return closedLoopVolts.getValueAsDouble();
    }

    private double getFF() {
        return 0;
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position, 0);
    }

    @Override
    public void createSelfTest() {

    }

    public boolean atPoint(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    @Override
    public boolean CAN_MOVE() {
        return true;
    }

    @Override
    public void periodic() {
        MALog.log(LOG_PATH + "PID Voltage", getCloseLoopVolts());
        MALog.log(LOG_PATH + "Absolute Position", absPosition.getValueAsDouble());
        super.periodic();
    }

    public static SixBar getInstance() {
        if (sixbar == null) {
            sixbar = new SixBar();
        }
        return sixbar;
    }
}
