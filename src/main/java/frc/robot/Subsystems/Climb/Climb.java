
package frc.robot.Subsystems.Climb;

import com.MAutils.Components.MACam;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.PortMap;
import frc.robot.Subsystems.SixBar.SixBar;

public class Climb extends PositionControlledSystem{

    private static Climb climb;

    private MACam macam = new MACam(PortMap.ClimbPorts.MACAM);

    private Climb() {
        super(ClimbConstnats.CLIMB_CONSTANTS, ClimbConstnats.IDLE, ClimbConstnats.CLIMB, ClimbConstnats.DOWN, ClimbConstnats.PRECLIMB);
    }

    @Override
    public void createSelfTest() {
        
    }

    @Override
    public boolean CAN_MOVE() {
        return (!DriverStation.isAutonomous() || (DriverStation.isAutonomous() && macam.getDistance() < ClimbConstnats.AUTONOMOUS_MIN_DISTANCE)) && SixBar.getInstance().getPosition() > ClimbConstnats.INTAKE_OPEN_POSITION;
    }

    public static Climb getInstance(){
        if(climb == null){
            climb = new Climb();
        }
        return climb;
    }
}
