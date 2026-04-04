
package frc.robot.Subsystems.Climb;

import com.MAutils.Components.MACam;
import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SixBar.SixBar;

public class Climb extends PositionControlledSystem {

    private static Climb climb;

    public static final State HOMING = new State("HOMING");
    public static final State PRECLIMB = new State("PRECLIMB");
    public static final State CLIMB = new State("CLIMB");

    private double climbAngle = 0;

    private DigitalInput ir = new DigitalInput(PortMap.ClimbPorts.IR);

    private Climb() {
        super(ClimbConstnats.CLIMB_CONSTANTS, ClimbConstnats.IDLE, CLIMB, ClimbConstnats.DOWN,
                PRECLIMB, HOMING);

        // ClimbConstnats.PRECLIMB.setOnStateSet(() -> setBrakeMode(false));
        // ClimbConstnats.DOWN.setOnStateSet(() -> setBrakeMode(false));
        // ClimbConstnats.CLIMB.setOnStateSet(() -> setBrakeMode(true));

        HOMING.setOnStateSet(() -> setConstants(ClimbConstnats.HOMING_CLIMB_CONSTANTS, false));
        HOMING.setOnStateEnd(() -> setConstants(ClimbConstnats.CLIMB_CONSTANTS, false));
        CLIMB.setOnStateSet(() -> setBrakeMode(true));
        PRECLIMB.setOnStateSet(() -> setBrakeMode(false));
        resetPosition(0);
    }

    @Override
    public void createSelfTest() {

    }

    @Override
    public boolean CAN_MOVE() {
        return SixBar.getInstance().getPosition() < ClimbConstnats.INTAKE_OPEN_POSITION;
    }// (DriverStation.isAutonomous() && getCurrentState() == ClimbConstnats.PRECLIMB
     // || getCurrentState() == ClimbConstnats.IDLE) || (DriverStation.isAutonomous()
     // && macam.getDistance() < ClimbConstnats.AUTONOMOUS_MIN_DISTANCE) &&

    public boolean isOnBar() {
        // return macam.getDistance() < ClimbConstnats.AUTONOMOUS_MIN_DISTANCE;
        return false;
    }

    @Override
    public void periodic() {
        super.periodic();

        MALog.log(LOG_PATH + "IR", getIR());
    }

    public boolean getIR() {
        return !ir.get();
    }

    public boolean atPoint(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }
        return climb;
    }
}
