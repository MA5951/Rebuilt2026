
package frc.robot.Util;

import com.MAutils.Logger.MALog;
import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotControl.Dashboard;
import edu.wpi.first.wpilibj.Timer;

public class ActiveUtil {

    private static Boolean isMyFirstShift; 
    private static Timer matchTimer = new Timer();
    private static int currentShift = 0;
    private static boolean isActiveDisabled = false;

    public static void startTeleop() {
        matchTimer.reset();
        matchTimer.start();
    }


    public static void setIsMyFirstShift(Boolean bool) {
        isMyFirstShift = bool;
    }

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
        if (isActiveDisabled) {
            return true;
        }

        return matchTimer.get() < 10 || matchTimer.get() > 110 || isMyShift();
    }

    public static double getTimeInActive() {
        if (isActiveDisabled) {
            return 0;
        }


        if(isMyFirstShift != null) {
            if (isMyFirstShift) {
                if (matchTimer.get() < 35) {                    
                    return 35 - matchTimer.get();
                } else if (matchTimer.get() > 60 && matchTimer.get() < 85) {
                    return 25 - (matchTimer.get() - 60);
                } else if(matchTimer.get() > 110) {
                    return 30 - (matchTimer.get() - 110);
                } 
            } else {
                if(matchTimer.get() < 10) {
                    return 10 - matchTimer.get();
                } else if ((matchTimer.get() > 35 && matchTimer.get() < 60)) {
                    return 25 - (matchTimer.get() - 35);
                } else if (matchTimer.get() > 85) {
                    return 55 - (matchTimer.get() - 85);
                }
            }
        }
        return -1;   
    }

    public static double getTimePastActive() {
        if (isActiveDisabled) {
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
        if (isActiveDisabled) {
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

    public static void getGameMode() {
        if (matchTimer.get() < 10) {
            MALog.log("/ActiveUtil/GameMode", "Transfer" );
            checkShift();
        } else if (matchTimer.get() > 110) {
            MALog.log("/ActiveUtil/GameMode", "Endgame" );
        } else if (isMyShift()) {
            MALog.log("/ActiveUtil/GameMode", "Active -  Shift: " + getCurrentShift());
        } else if(!isMyShift()) {
            MALog.log("/ActiveUtil/GameMode", "Inactive -  Shift: " + getCurrentShift());
        } 
    }

    public static int getCurrentShift() {
        if (matchTimer.get() < 35 && matchTimer.get() > 10) {
            currentShift = 1;
        } else if (matchTimer.get() > 35 && matchTimer.get() < 60) {
            currentShift = 2;
        } else if (matchTimer.get() > 60 && matchTimer.get() < 85) {
            currentShift = 3;
        } else if (matchTimer.get() > 85 && matchTimer.get() < 110) {
            currentShift = 4;
        } else {
            currentShift = 0; 
        }
        return currentShift;
    }

    public static void setActiveDisabled(boolean activeDisabled) {
        isActiveDisabled = activeDisabled;
    }

    public static boolean isActiveDisabled() {
        return isActiveDisabled;
    }

    public static double getMatchTime() {
        return matchTimer.get();
    }

    public static boolean isMyFirstShift() {
        return isMyFirstShift;
    }
} 