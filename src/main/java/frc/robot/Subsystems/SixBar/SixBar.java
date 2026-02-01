
package frc.robot.Subsystems.SixBar;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;


public class SixBar extends PositionControlledSystem {

    private static SixBar sixbar;

    private StatusSignal<Double> closedLoopVolts;

    //TODO add cancoder

    private SixBar() {
        super(SixBarConstants.SIXBAR_CONSTANTS, SixBarConstants.ARMBRAKS, SixBarConstants.COLLISION,
                SixBarConstants.SHOOTING, SixBarConstants.IDLE, SixBarConstants.DEPLOY);

        closedLoopVolts = systemIO.getSystemConstants().master.motorController.getClosedLoopOutput();

        StatusSignalsRunner.registerSignals(false, closedLoopVolts); //TODO change to the master.canbusID 
    }

    //TODO need to add reast from cancoder  


    public double getCloseLoopVolts() {
        return closedLoopVolts.getValueAsDouble();
    }

    //TODO need to add FF func 
        
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
