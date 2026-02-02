
package frc.robot.RobotControl;

import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultSuperStructure;
import com.MAutils.Utils.ChassisSpeedsUtil;
import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.BooleanLatch;
import frc.robot.Util.Field;
import frc.robot.Util.GeometryUtil;
import frc.robot.Util.ShootingParameters;
import frc.robot.Util.GeometryUtil.Result;

public class SuperStructure extends DeafultSuperStructure {
    public static final Translation2d FEEDING_POSE = new Translation2d(2, 2);

    public static final double IN_THE_AIR_CURRENT_THRESHOLD = 40.0;
    public static final double CLIMB_POSITION_THRESHOLD = 0.15;

    public static final double FEEDING_ANGLE_OFFSET = 20;
    public static final double FEEDING_SHOOTER_OFFSET = 20;

    public static final double FEEDING_IN_MOTION_FIELD_MARGIN = 1;
    public static final double FEEDING_IN_MOTION_NET_MARGIN = 0.5;
    public static final double FEEDING_IN_MOTION_MIN_DISTANCE = 1;

    public static final double SANDWICH_STUCK_DELTA = 10;

    private static Result lastFeedingResult = new Result(false, -1, new Translation2d());
    private static ShootingParameters currentShootingParameters;
    private static boolean automatic = true;
    private static boolean defence = false;

    private static Debouncer inTheAirDebouncer = new Debouncer(0.8);
    private static Debouncer sandwichStuckDebouncer = new Debouncer(0.6);

    private static BooleanLatch atPointLatch = new BooleanLatch();

    public enum ShootingPreset {
        CLOSE(10.0, 2000.0, new Pose2d()),
        CLIMB(15.0, 2500.0, new Pose2d());

        public final double hoodAngle;
        public final double shooterRPM;
        public final Pose2d pose;

        private ShootingPreset(double hoodAngle, double shooterRPM, Pose2d pose) {
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
        super(() -> ChassisSpeedsUtil.getSpeedMagnitude(Swerve.getInstance().getChassisSpeeds()));
    }

    public static boolean isMainTag() {
        return Vision.getInstance().isMainTag();
    }

    public static double getRelAngleToTarget() {
        return 0.0;
    }

    public static double getAbsAngleToTarget() {
        return GeometryUtil.angleTo(PoseEstimator.getCurrentPose(), Field.getHub());
    }

    public static double getAngleToFeeding() {
        return GeometryUtil.angleTo(PoseEstimator.getCurrentPose(), FEEDING_POSE);
    }

    public static boolean isHittingNet() {
        return GeometryUtil.willShotHitNet(PoseEstimator.getCurrentPose(), ShooterConstants.SHOOTER_OFFSET,
                Swerve.getInstance().getRobotRotation2d(), Field.getNetA(),
                Field.getNetB(),
                20, FEEDING_IN_MOTION_NET_MARGIN);
    }

    public static boolean outSideField() {
        return lastFeedingResult.valid && Field.FIELD_RECTANGLE.contains(lastFeedingResult.hitPointField)
                && lastFeedingResult.distanceMeters > FEEDING_IN_MOTION_MIN_DISTANCE;
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
        return !(isStuck() == StuckType.NONE);
    }

    public static double getTimeLeft() {
        return DriverStation.getMatchTime();
    }

    public static boolean isRobotInAir() {
        return inTheAirDebouncer.calculate(Climb.getInstance().getCurrent() > IN_THE_AIR_CURRENT_THRESHOLD) && Climb.getInstance().getPosition() < CLIMB_POSITION_THRESHOLD;
    }

    public static StuckType isStuck() {
        if (Sandwich.getInstance().isMoving() && isBallsInSandwich()
                && sandwichStuckDebouncer.calculate(Sandwich.getInstance().getDeltaMAcamDistance() < SANDWICH_STUCK_DELTA)) {
            return StuckType.STUCK_IN_SANDWICH;
        } else if (!isBallsInSandwich() && Roller.getInstance().isMoving() && Transfer.getInstance().isMoving()
                && isBalls()) {
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
        return new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()) + FEEDING_SHOOTER_OFFSET,
                getHoodAngle(getDistanceToTargetFeeding()) + FEEDING_ANGLE_OFFSET);
    }

    private static double getDistanceToTargetShooting() {
        return isMainTag() ? Math.pow(Vision.getInstance().getDistanceTryg(), 2) + Math.pow(Field.HUB_WIDTH / 2, 2)
                + 2 * Vision.getInstance().getDistanceTryg() * (Field.HUB_WIDTH / 2) * Math.cos(Math.toRadians(
                        VisionConstants.FRONT_LL.getCameraIO().getTag().txnc + Swerve.getInstance().getGyroData().yaw))
                : GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(), VisionConstants.FRONTLL_OFFSET)
                    .getDistance(Field.getHub());
    }

    private static double getDistanceToTargetFeeding() {
        if (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            lastFeedingResult = GeometryUtil.distanceToVerticalLineX(
                    PoseEstimator.getCurrentPose(), VisionConstants.FRONTLL_OFFSET,
                    Swerve.getInstance().getRobotRotation2d(), Field.getAllianceXLine());
            return lastFeedingResult.distanceMeters;
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING) {
            return GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(), VisionConstants.FRONTLL_OFFSET)
                    .getDistance(FEEDING_POSE);
        }

        return -1;
    }

    public static boolean isInTheAlinceZone() {
        return DriverStationUtil.getAlliance() == Alliance.Blue
                ? PoseEstimator.getCurrentPose().getX() < Field.ALLIANCE_WIDTH
                : PoseEstimator.getCurrentPose().getX() > Field.LENGTH - Field.ALLIANCE_WIDTH;
    }
    
    public static boolean atPointForShooting() {
        //return atPointLatch.calculate((Swerve.getInstance().atPointForShooting() || !isAutomatic()) && Shooter.getInstance().atPointForShooting() && Hood.getInstance().atPointForShooting()); //Full Latch

        return atPointLatch.calculate(Shooter.getInstance().atPointForShooting()) && Hood.getInstance().atPointForShooting() && (Swerve.getInstance().atPointForShooting() || !isAutomatic()) ; //Intiligent Latch

        // (Swerve.getInstance().atPointForShooting() || !isAutomatic()) && Shooter.getInstance().atPointForShooting() && Hood.getInstance().atPointForShooting();//No Latch
    }

    public static boolean atPointForFeeding() {
        //return atPointLatch.calculate((Swerve.getInstance().atPointForFeeding() || !isAutomatic()) && Shooter.getInstance().atPointForFeeding() && Hood.getInstance().atPointForFeeding()); //Full Latch

        return atPointLatch.calculate(Shooter.getInstance().atPointForFeeding()) && Hood.getInstance().atPointForFeeding() && (Swerve.getInstance().atPointForFeeding() || !isAutomatic()) ; //Intiligent Latch

        //return (Swerve.getInstance().atPointForFeeding() || !isAutomatic()) && Shooter.getInstance().atPointForFeeding() && Hood.getInstance().atPointForFeeding(); //No Latch
    }

    public static boolean atPointForFeedingInMotion() {
        //return atPointLatch.calculate((Swerve.getInstance().atPointForFeedingInMotion() || !isAutomatic()) && Shooter.getInstance().atPointForFeedingInMotion() && Hood.getInstance().atPointForFeedingInMotion()); //Full Latch

        return atPointLatch.calculate(Shooter.getInstance().atPointForFeedingInMotion()) && Hood.getInstance().atPointForFeedingInMotion() && (Swerve.getInstance().atPointForFeedingInMotion() || !isAutomatic()) ; //Intiligent Latch

        //return (Swerve.getInstance().atPointForFeedingInMotion() || !isAutomatic()) && Shooter.getInstance().atPointForFeedingInMotion() && Hood.getInstance().atPointForFeedingInMotion(); //No Latch
    }

    public static void update() {
        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING) {    
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetShooting()),
                    getHoodAngle(getDistanceToTargetShooting()));
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING
                || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()),
                    getHoodAngle(getDistanceToTargetFeeding()));
        }
    }



}
