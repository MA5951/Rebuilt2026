

package frc.robot.Subsystems.Hood;


import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.DashBoard.Tunable;
import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import frc.robot.PortMap;

public class Hood extends PositionControlledSystem {

    private static Hood hood;
    private Tunable hoodPosition;

    public static final State HOMING = new State("HOMING");

    private Hood() {
        super(HoodConstants.HOOD_CONSTANTS, HoodConstants.IDLE, HoodConstants.EJECT, HoodConstants.FEEDING,
                HoodConstants.SHOOTING, HoodConstants.FEEDING_IN_MOTION, HOMING);

        hoodPosition = new Tunable(0, "Hood Position");

        HOMING.setOnStateSet(() -> setConstants(HoodConstants.HOOD_CONSTANTS_HOMING, false));
        HOMING.setOnStateEnd(() -> setConstants(HoodConstants.HOOD_CONSTANTS, false));
    }

    public boolean atPointForShooting() {
        return atPoint();
    }

    public boolean atPointForFeeding() {
        return true;
    }

    public boolean atPointForFeedingInMotion() {
        return true;
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

    public double getHoodPosition() {
        return hoodPosition.get();
    }

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }
}
