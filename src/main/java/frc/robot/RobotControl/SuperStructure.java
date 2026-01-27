
package frc.robot.RobotControl;

import com.MAutils.RobotControl.DeafultSuperStructure;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.ShootingParameters;

public class SuperStructure extends DeafultSuperStructure{ 
    private static ShootingParameters currentShootingParameters;

    public enum ShootingPreset {
        CLOSE(10.0, 2000.0, new Pose2d()),
        CLIMB(15.0, 2500.0, new Pose2d());

        public final double hoodAngle;
        public final double shooterRPM;
        public final Pose2d pose;

        private ShootingPreset (double hoodAngle, double shooterRPM, Pose2d pose) {
            this.hoodAngle = hoodAngle;
            this.shooterRPM = shooterRPM;
            this.pose = pose;
        }
    }

    public enum StuckType {
        NONE,
        STUCK_IN_SANDWICH,
        STUCK_IN_TRANSFER;
    }

    public SuperStructure() {
        super( () -> Swerve.getInstance().getChassisSpeeds());
    }

    public static int getMainTagID() {
        return 0;
    }

    public static double getTxToTarget() {
        return 0.0;
    }

    public static double getAbsAngleToTarget() {
        return 0.0;
    }

    public static double getAngleToFeeding() {
        return 0.0;
    }

    public static double getDistanceToFeeding() {
        return 0.0;
    }

    public static boolean isHittingNet() {
        return false;
    }

    public static boolean isFull() {
        return false;
    }

    public static boolean isBalls() {
        return false;
    }

    public static boolean isBallsInSandwich() {
        return Sandwich.getMacam < ;
    }

    public static StuckType isStuck() {
        if (Sandwich.isMoving() && isBallsInSandwich() && Sandwich.getMAcamDelta < 1) {
            return StuckType.STUCK_IN_SANDWICH;
        } else if (!isBallsInSandwich() && Roller.isMoving() && Transfer.isMoving && isBalls()) {
            return StuckType.STUCK_IN_TRANSFER;
        } else {
            return StuckType.NONE;
        }
    }

    private static double getShootingRPM(double distance) {
        return 0.0;
    } 

    private static double getHoodAngle(double distance) {
        return 0.0;
    }

    public static ShootingParameters getShootingParameters() {
        return currentShootingParameters;
    }

    public static ShootingParameters getFeedingParameters() {
        return new ShootingParameters(getShootingRPM(0.0), getHoodAngle(0.0));
    }

    private static double getDistanceToTargetShooting() {
        return 0.0;
    }

    private static double getDistanceToTargetFeeding() {
        return 0.0;
    }

    public static void update() {
        if (RobotContainer.getRobotState() == SHOOTING) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetShooting()),  getHoodAngle(getDistanceToTargetShooting()));
        } else if (RobotContainer.getRobotState() == FEEDING || RobotContainer.getRobotState() == FEEDING_IN_MOTION ) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()),  getHoodAngle(getDistanceToTargetFeeding()));
        }
    }









}
