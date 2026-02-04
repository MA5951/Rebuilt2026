
package frc.robot.RobotControl;

import com.MAutils.DashBoard.Tunable;

public class Dashboard {
    private static Tunable activeDisable = new Tunable(0, "/Dashboard/ActiveDisabled"); //TODO move to constructor
    //TODO what about all the over things we talked about? like befor and after avtive in rumbell
    //TODO if we can go under the tranch and ofher things we talked about in the driver talk

    public Dashboard() {
    
    }

    public static boolean isActiveDisabled() {
        return activeDisable.get() == 1;
    }
}
