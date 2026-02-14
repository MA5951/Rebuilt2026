

package frc.robot.Subsystems.Hood;


import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.DashBoard.Tunable;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import frc.robot.PortMap;
import frc.robot.Subsystems.Swerve.Swerve;

public class Hood extends PositionControlledSystem {

    private static Hood hood;
    private StatusSignal<Angle> absPosition;
    private final CANcoder canCoder;
    private Tunable hoodPosition;

    private Hood() {
        super(HoodConstants.HOOD_CONSTANTS, HoodConstants.IDLE, HoodConstants.EJECT, HoodConstants.FEEDING,
                HoodConstants.SHOOTING, HoodConstants.FEEDING_IN_MOTION);

        canCoder = new CANcoder(PortMap.HoodPorts.CAN_CODER, PortMap.CAN_BUS.CANIVORE_BUS);

        absPosition = canCoder.getAbsolutePosition();
        absPosition.refresh();

        // resetPosition((absPosition.getValueAsDouble() * 360) / HoodConstants.CAN_CODER_GEAR);
        resetPosition(0);

        StatusSignalsRunner.registerSignals(PortMap.HoodPorts.HOOD_MOTOR, absPosition);

        hoodPosition = new Tunable(0, "Hood Position");
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

    @Override
    public void periodic() {
        MALog.log(LOG_PATH + "Absolute Position", absPosition.getValueAsDouble() * 360 / HoodConstants.CAN_CODER_GEAR);
        super.periodic();
    }

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }
}
