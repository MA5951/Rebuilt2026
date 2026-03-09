
package frc.robot.RobotControl;

import com.MAutils.DashBoard.Tunable;
import com.MAutils.DashBoard.TunableBoolean;

public class Dashboard {
    private static TunableBoolean activeDisable = new TunableBoolean(false, "/Dashboard/ActiveDisabled");
    private static TunableBoolean isFirstShift = new TunableBoolean(false, "/Dashboard/Is First Shift");
    private static TunableBoolean isHoodStuck = new TunableBoolean(false, "/Dashboard/Is Hood Stuck");
    private static TunableBoolean isDefenceMode = new TunableBoolean(false, "/Dashboard/Is Defence Mode");
    private static Tunable shooterFactor = new Tunable(1, "/Dashboard/Shooter Factor");

    public Dashboard() {
        
    
    }

    public static boolean isActiveDisabled() {
        return activeDisable.get();
    }

    public static boolean isFirstShift() {
        return isFirstShift.get();
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
