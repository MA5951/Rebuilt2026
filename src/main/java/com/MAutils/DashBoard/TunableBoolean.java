
package com.MAutils.DashBoard;

import com.MAutils.Logger.MALog;
import com.MAutils.Utils.Constants;


/*
 * Represents a tunable parameter that can be adjusted via a dashboard when in tuning mode.
 */
public class TunableBoolean {

    private final boolean defaultValue;
    private final String key;
    private boolean lastValue;

    public TunableBoolean(boolean defaultValue, String name) {
        this.defaultValue = defaultValue;
        this.key = "Tunable/" + name;
        if (Constants.TUNING_MODE) {
            MALog.log(key, defaultValue);
        }
        lastValue = defaultValue;
    }

    public boolean get() {
        if (Constants.TUNING_MODE) { 
            return MALog.get(key).getBoolean(defaultValue);
        }

        return defaultValue;
    }

    public boolean hasChnaged() {
        if (get() != lastValue) {
            lastValue = get();
            return true;
        }

        return false;
    }
}
