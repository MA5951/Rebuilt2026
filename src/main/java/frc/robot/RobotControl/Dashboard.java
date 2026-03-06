
package frc.robot.RobotControl;

import com.MAutils.DashBoard.Tunable;

public class Dashboard {
    private static Tunable activeDisable = new Tunable(0, "/Dashboard/ActiveDisabled");
    private static Tunable isFirstShift = new Tunable(0, "/Dashboard/Is First Shift");


    public Dashboard() {
        
    
    }

    public static boolean isActiveDisabled() {
        return activeDisable.get() == 1;
    }

    public static boolean isFirstShift() {
        return isFirstShift.get() == 1;
    }
}
