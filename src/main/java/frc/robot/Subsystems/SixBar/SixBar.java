
package frc.robot.Subsystems.SixBar;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

public class SixBar extends PositionControlledSystem {

    private static SixBar sixbar;

    private SixBar() {
        super(SixBarConstants.SIXBAR_CONSTANTS, SixBarConstants.ARMBRAKS, SixBarConstants.COLLISION,
                SixBarConstants.SHOOTING, SixBarConstants.IDLE, SixBarConstants.DEPLOY);
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
