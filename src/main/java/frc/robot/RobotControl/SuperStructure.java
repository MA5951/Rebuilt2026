
package frc.robot.RobotControl;

import com.MAutils.RobotControl.DeafultSuperStructure;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Util.ShootingParameters;

public class SuperStructure extends DeafultSuperStructure{ 
    private static ShootingParameters currentShootingParameters;
    private static boolean automatic = true;
    private static boolean defence = false;


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
        super( () -> Swerve.getInstance().get);
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

    public static boolean outSideField() {
        return false;
    }

    public static boolean isFull() {
        return false;
    }

    public static boolean isBalls() {
        return false;
    }

    public static boolean isBallsInSandwich() {
        return Sandwich.getInstance().getMACamDistance() < SandwichConstants.IS_BALLS_DISTANCE;
    }

    public static boolean isInActive() {
        return true;
    }

    public static double getTimePastActive() {
        return 0.0;
    }

    public static double getTimeUntilActive() {
        return 0.0;
    }

    public static boolean isAutomatic() {
        return automatic;
    }

    public static void setAutomatic(boolean update) {
        automatic = update;
    }

    public static boolean isDefenceMode() {
        return defence;
    }

    public static void setDefenceMode(boolean update) {
        defence = update;
    }

    public static boolean isTransferStuck() {
        return false;
    }

    public static double getTimeLeft() {
        return 9;
    }

    public static boolean isRobotInAir() {
        return false;
    }

    public static StuckType isStuck() {
        if (Sandwich.getInstance().isMoving() && isBallsInSandwich() && Sandwich.getInstance().getDeltaMAcamDistance() < 1) {
            return StuckType.STUCK_IN_SANDWICH;
        } else if (!isBallsInSandwich() && Roller.getInstance().isMoving() && Transfer.getInstance().isMoving && isBalls()) {
            return StuckType.STUCK_IN_TRANSFER;
        } else {
            return StuckType.NONE;
        }
    }


    public static boolean isDefenceMode() {
        return false;
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

    public static boolean isInTheAlinceZone() {
        return false;
    }

    public static void update() {
        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetShooting()),  getHoodAngle(getDistanceToTargetShooting()));
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION ) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()),  getHoodAngle(getDistanceToTargetFeeding()));
        }
    }









}
