
package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.SwerveSystem;

public class Swerve extends SwerveSystem{
    private static Swerve instance;

    private Swerve() {
        super(SwerveConstants.SWERVE_CONSTANTS);
        //TODO what about the in ramp posestimation considiration, need to stop relay on vision and the odometry 
        //TODO i write it here but i see that there are still commant for the cr of the mautils let go over them
    }


    public boolean atPointForShooting() {
        return true; //TODO imploment
    }

    public boolean atPointForFeeding() {
        return true; //TODO imploment
    }

    public boolean atPointForFeedingInMotion() {
        return true; //TODO imploment
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }



}
