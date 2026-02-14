
package frc.robot.RobotControl;

import com.MAutils.DashBoard.Tunable;

public class Dashboard {
    private static Tunable activeDisable = new Tunable(0, "/Dashboard/ActiveDisabled");


    public Dashboard() {
        
    
    }

    public static boolean isActiveDisabled() {
        return activeDisable.get() == 1;
    }
}
