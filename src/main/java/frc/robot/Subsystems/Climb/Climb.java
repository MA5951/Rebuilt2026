
package frc.robot.Subsystems.Climb;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

public class Climb extends PositionControlledSystem{

    private static Climb climb;

    //TODO need to add IR 

    private Climb() {
        super(ClimbConstnats.CLIMB_CONSTANTS, ClimbConstnats.IDLE, ClimbConstnats.CLIMB, ClimbConstnats.DOWN, ClimbConstnats.PRECLIMB);
    }

    @Override
    public void createSelfTest() {
        
    }

    @Override
    public boolean CAN_MOVE() {
        return true; //TODO cant open if the intake is close , and in auto cant close if the IR dont return true
    }

    public static Climb getInstance(){
        if(climb == null){
            climb = new Climb();
        }
        return climb;
    }
}
