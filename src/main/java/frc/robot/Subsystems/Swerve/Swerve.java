
package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.SwerveSystem;

public class Swerve extends SwerveSystem{
    private static Swerve instance;

    private Swerve() {
        super(SwerveConstants.SWERVE_CONSTANTS);
    }


    public boolean atPointForShooting() {
        return true;
    }

    public boolean atPointForFeeding() {
        return true;
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }



}
