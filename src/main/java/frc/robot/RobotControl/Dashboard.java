
package frc.robot.RobotControl;

import com.MAutils.DashBoard.Tunable;
import com.MAutils.DashBoard.TunableBoolean;

public class Dashboard {
    private static Tunable activeDisable = new Tunable(0, "/Dashboard/ActiveDisabled");
    private static Tunable isFirstShift = new Tunable(0, "/Dashboard/Is First Shift");
    private static TunableBoolean isHoodStuck = new TunableBoolean(false, "/Dashboard/Is Hood Stuck");
    private static TunableBoolean isDefenceMode = new TunableBoolean(false, "/Dashboard/Is Defence Mode");
    private static Tunable shooterFactor = new Tunable(1, "/Dashboard/Shooter Factor");

    public Dashboard() {
        
    
    }

    public static boolean isActiveDisabled() {
        return activeDisable.get() == 1;
    }

    public static boolean isFirstShift() {
        return isFirstShift.get() == 1;
    }

    public static boolean isHoodStuck() {
        return isHoodStuck.get();
    }

    public static boolean isDefenceMode() {
        return isDefenceMode.get();
    }

    public static double getShooterFactor() {
        return shooterFactor.get();
    }
}
