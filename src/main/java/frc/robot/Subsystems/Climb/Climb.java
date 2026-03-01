
package frc.robot.Subsystems.Climb;

import com.MAutils.Components.MACam;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

import frc.robot.Subsystems.SixBar.SixBar;

public class Climb extends PositionControlledSystem{

    private static Climb climb;

    //private MACam macam = new MACam(PortMap.ClimbPorts.MACAM);

    private Climb() {
        super(ClimbConstnats.CLIMB_CONSTANTS, ClimbConstnats.IDLE, ClimbConstnats.CLIMB, ClimbConstnats.DOWN, ClimbConstnats.PRECLIMB, ClimbConstnats.HOMING);
        resetPosition(0);
    }

    @Override
    public void createSelfTest() {
        
    }

    @Override
    public boolean CAN_MOVE() {
        return SixBar.getInstance().getPosition() < ClimbConstnats.INTAKE_OPEN_POSITION;
    }//(DriverStation.isAutonomous() && getCurrentState() == ClimbConstnats.PRECLIMB || getCurrentState() == ClimbConstnats.IDLE) || (DriverStation.isAutonomous() && macam.getDistance() < ClimbConstnats.AUTONOMOUS_MIN_DISTANCE) && 

    public boolean isOnBar() {
        //return macam.getDistance() < ClimbConstnats.AUTONOMOUS_MIN_DISTANCE;
        return false;
    }


    public boolean atPoint(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    public static Climb getInstance(){
        if(climb == null){
            climb = new Climb();
        }
        return climb;
    }
}
