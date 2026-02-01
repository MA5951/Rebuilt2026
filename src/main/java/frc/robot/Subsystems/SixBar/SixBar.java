
package frc.robot.Subsystems.SixBar;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;


public class SixBar extends PositionControlledSystem {

    private static SixBar sixbar;

    private StatusSignal<Double> closedLoopVolts;

    private SixBar() {
        super(SixBarConstants.SIXBAR_CONSTANTS, SixBarConstants.ARMBRAKS, SixBarConstants.COLLISION,
                SixBarConstants.SHOOTING, SixBarConstants.IDLE, SixBarConstants.DEPLOY);

        closedLoopVolts = systemIO.getSystemConstants().master.motorController.getClosedLoopOutput();

        StatusSignalsRunner.registerSignals(false, closedLoopVolts);
    }


    public double getCloseLoopVolts() {
        return closedLoopVolts.getValueAsDouble();
    }
        
    @Override
    public void createSelfTest() {

    }

    @Override
    public boolean CAN_MOVE() {
        return true;
    }

    

    public static SixBar getInstance() {
        if (sixbar == null) {
            sixbar = new SixBar();
        }
        return sixbar;
    }
}
