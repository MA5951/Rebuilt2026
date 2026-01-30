
package frc.robot.Util;

import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class ActiveUtil {

    private static Boolean isFirstShift;
    private static Timer matchTimer = new Timer();

    public static void startTeleop() {
        matchTimer.reset();
        matchTimer.start();
    }

    public static void checkShift() {
        if (isFirstShift == null && DriverStation.getGameSpecificMessage().length() > 0) {
            if (DriverStationUtil.getAlliance() == Alliance.Red
                    && DriverStation.getGameSpecificMessage().charAt(0) == 'B') {
                isFirstShift = true;
            } else if (DriverStationUtil.getAlliance() == Alliance.Blue
                    && DriverStation.getGameSpecificMessage().charAt(0) == 'R') {
                isFirstShift = true;
            } else {
                isFirstShift = false;
            }
        }
    }

    private static boolean isMyShift() {
        if (isFirstShift != null) {
            if (isFirstShift) {
                return matchTimer.get() < 35 || (matchTimer.get() > 60 && matchTimer.get() < 85);
            } else {
                return (matchTimer.get() > 35 && matchTimer.get() < 60) || matchTimer.get() > 85;
            }
        }
        return false;
    }

    public static boolean isActive() {
        return matchTimer.get() < 10 || matchTimer.get() > 110 || isMyShift();
    }

    public static double getTimeInActive() {
        if(isFirstShift != null) {
            if (isFirstShift) {
                if (matchTimer.get() < 35) {
                    return matchTimer.get();
                } else if (matchTimer.get() > 60 && matchTimer.get() < 85) {
                    return matchTimer.get() - 60;
                } else if(matchTimer.get() > 110) {
                    return matchTimer.get() - 110;
                } 
            } else {
                if(matchTimer.get() < 10) {
                    return matchTimer.get();
                } else if ((matchTimer.get() > 35 && matchTimer.get() < 60)) {
                    return matchTimer.get() - 35;
                } else if (matchTimer.get() > 85) {
                    return matchTimer.get() - 85;
                }
            }
        }
        return -1;   
    }

    public static double getTimePastActive() {
        if(isFirstShift != null) {
            if (isFirstShift) {
                if (isActive()) {
                    return -1;
                } else if (matchTimer.get() < 60) {
                    return matchTimer.get() - 35;
                } else if (matchTimer.get() < 110) {
                    return matchTimer.get() - 85;
                } 
            } else {
                if (isActive()) {
                    return -1;
                } else if (matchTimer.get() < 35) {
                    return matchTimer.get() - 10;
                } else if (matchTimer.get() < 85) {
                    return matchTimer.get() - 60;
                } 
            }
        }
        return -1;   
    }

    public static double getTimeUntilActive() {
        if(isFirstShift != null) {
            if (isFirstShift) {
                if (isActive()) {
                    return -1;
                } else if (matchTimer.get() < 60) {
                    return 60 - matchTimer.get();
                } else if(matchTimer.get() < 110) {
                    return 110 - matchTimer.get();
                } else {
                    return -1;
                }
            } else {
                if(isActive()) {
                    return -1;
                } else if (matchTimer.get() < 35) {
                    return 35 - matchTimer.get();
                } else if (matchTimer.get() < 85) {
                    return 85 - matchTimer.get();
                } else {
                    return -1;
                }
            }
        }
        return -1;   
    }
} 