

package frc.robot.Subsystems.Hood;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

public class Hood extends PositionControlledSystem {

    private static Hood hood;

    private Hood() {
        super(HoodConstants.HOOD_CONSTANTS, HoodConstants.IDLE, HoodConstants.EJECT, HoodConstants.FEEDING,
                HoodConstants.SHOOTING, HoodConstants.FEEDING_IN_MOTION);

    }

    @Override
    public void createSelfTest() {

    }

    @Override
    public boolean CAN_MOVE() {
        return true;
    }

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }
}
