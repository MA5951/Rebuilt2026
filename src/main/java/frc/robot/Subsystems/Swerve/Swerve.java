
package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.SwerveSystem;


public class Swerve extends SwerveSystem{
    private static Swerve instance;

    private boolean isRampFlag = false;

    private Swerve() {
        super(SwerveConstants.SWERVE_CONSTANTS);

        
    }


    public boolean atPointForShooting() {
        return true;
    }

    public boolean atPointForFeeding() {
        return true;
    }

    public boolean atPointForFeedingInMotion() {
        return true;
    }

    public double getVelocityVector() {
        return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(getChassisSpeeds().vyMetersPerSecond,2));
    }


    @Override
    public void periodic() {
        super.periodic();

        if (Math.abs(getGyroData().roll) > 8 || Math.abs(getGyroData().pitch) > 8) {
            isRampFlag = true;
        }
        

    }

    public void resetRampFlag() {
        isRampFlag = false;
    }


    public boolean isRampFlag() {
        return isRampFlag;
    }
   

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }



}
