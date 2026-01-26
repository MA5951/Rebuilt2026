
package com.MAutils.RobotControl;

/*
 * Represents a state for a specific subsystem of the robot.
 */
public class State {

    private final String stateName; 
    private StateSubsystem subsystem;
    private Runnable onStateSet = () -> {
    };

    public State(String state_name, Runnable onStateSet, StateSubsystem subsystem) {
        this.stateName = state_name;
        this.subsystem = subsystem;
        this.onStateSet = onStateSet;
    }

    public State(String state_name, StateSubsystem subsystem) {
        this.stateName = state_name;
        this.subsystem = subsystem;
    }

    public State(String state_name) {
        this.stateName = state_name;
    }

    public void setSystem(StateSubsystem subsystem) {
        if (this.subsystem == null) {
            this.subsystem = subsystem;
        } else {
            throw new IllegalArgumentException("Subsystem already set for state " + stateName);
        }
    }

    public StateSubsystem getSubsystem() {
        return subsystem;
    }

    public void runRunnable() {
        onStateSet.run();
    }

}
