
package frc.robot.Util;


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
    }

}
