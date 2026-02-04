
package frc.robot.Util;

import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotControl.Dashboard;
import edu.wpi.first.wpilibj.Timer;

public class ActiveUtil {

    private static Boolean isMyFirstShift; 
    private static Timer matchTimer = new Timer(); //TODO why ust the timer and dont use the DriverStation match time


    public static void startTeleop() {
        matchTimer.reset();
        matchTimer.start();
    }

    //TODO you dont consider the dashboard override here
    //TODO also what about the code that get the data from the fms

    public static void checkShift() {
        if (isMyFirstShift == null && DriverStation.getGameSpecificMessage().length() > 0) {
            if (DriverStationUtil.getAlliance() == Alliance.Red
                    && DriverStation.getGameSpecificMessage().charAt(0) == 'B') {
                isMyFirstShift = true;
            } else if (DriverStationUtil.getAlliance() == Alliance.Blue
                    && DriverStation.getGameSpecificMessage().charAt(0) == 'R') {
                isMyFirstShift = true;
            } else {
                isMyFirstShift = false;
            }
        }
    }

    private static boolean isMyShift() {
        if (isMyFirstShift != null) {
            if (isMyFirstShift) {
                return matchTimer.get() < 35 || (matchTimer.get() > 60 && matchTimer.get() < 85);
            } else {
                return (matchTimer.get() > 35 && matchTimer.get() < 60) || matchTimer.get() > 85;
            }
        }
        return true;
    }

    public static boolean isActive() {
        if (Dashboard.isActiveDisabled()) {
            return true;
        }

        return matchTimer.get() < 10 || matchTimer.get() > 110 || isMyShift();
    }

    public static double getTimeInActive() {
        if (Dashboard.isActiveDisabled()) {
            return 0;
        }


        if(isMyFirstShift != null) {
            if (isMyFirstShift) {
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
        if (Dashboard.isActiveDisabled()) {
            return 0;
        }

        if(isMyFirstShift != null) {
            if (isMyFirstShift) {
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
        if (Dashboard.isActiveDisabled()) {
            return 0;
        }

        if(isMyFirstShift != null) {
            if (isMyFirstShift) {
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