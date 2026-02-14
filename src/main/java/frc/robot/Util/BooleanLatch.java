
package frc.robot.Util;

import com.MAutils.Logger.MALog;

public class BooleanLatch {


    private boolean value;

    public BooleanLatch() {
        value = false;
    }

    public boolean calculate(boolean input) {
        if (input) {
            value = true;
        } 
        return value;
    }

    public void reset() {
        value = false;
        MALog.log("/SuperStructure/Boolean Latch", "Latch");
    }

}
