
package frc.robot.RobotControl;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultSuperStructure;
import com.MAutils.Utils.ChassisSpeedsUtil;
import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Commands.SwerveController;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.ActiveUtil;
import frc.robot.Util.Field;
import frc.robot.Util.GeometryUtil;
import frc.robot.Util.ShootingParameters;
import frc.robot.Util.InterpolationTable;

public class SuperStructure extends DeafultSuperStructure {
    public static final Translation2d FEEDING_POSE = Field.flipByAlliance(new Translation2d(Field.LENGTH - 1.3, 3.6));

    public static final double IN_THE_AIR_CURRENT_THRESHOLD = 40.0;
    public static final double CLIMB_POSITION_THRESHOLD = 0.15;

    public static final double FEEDING_ANGLE_OFFSET = 0;
    public static final double FEEDING_SHOOTER_OFFSET = -700;
    public static final double FEEDING_DISTANCE_OFFSET = -2;

    public static final double FEEDING_IN_MOTION_FIELD_MARGIN = 1.2;// 1.3
    public static final double FEEDING_IN_MOTION_NET_MARGIN = 0;
    public static final double FEEDING_IN_MOTION_MIN_DISTANCE = 1;

    public static final double SANDWICH_STUCK_DELTA = 10;
    public static  double shootingFactor = 1;

    public static double AFTER_ANGLE = 0;
    //this class is ugly becuase of galdo

    private static boolean isExtendedMagazine = true;

    private static double[][] hoodTableData = {
            { 5.676, 27.0 },
            { 5.436, 27.0 },
            { 5.126, 26.0 },
            { 4.996, 25.0 },
            { 4.816, 25.0 },
            { 4.616, 25.0 },
            { 4.446, 25.0 },
            { 4.276, 22.0 },
            { 3.936, 22.0 },
            { 3.756, 22.0 },
            { 3.587, 21.0 },
            { 3.428, 20.0 },
            { 3.244, 19.5 },
            { 3.109, 17.5 },
            { 2.986, 17 },
            { 2.892, 15.5 },
            { 2.795, 15.5 },
            { 2.691, 14.5 },
            { 2.551, 14.5 },
            { 2.468, 14.0 },
            { 2.374, 14.0 },
            { 2.278, 13.0 },
            { 2.184, 13.0 },
            { 2.090, 13.0 },
            { 2.014, 13.0 },
            { 1.887, 11.0 },
            { 1.783, 10.0 },
            { 1.697, 10.0 },
            { 1.591, 9.80 },
            { 1.514, 9.8 },
            { 1.344, 9.8 },
            { 1.2, 8.5 },
            { 1, 8.5 },
    };

    private static double[][] shooterTableData = {
            { 5.676, 4150.0 },
            { 5.436, 4055.0 },
            { 5.216, 4000.0 },
            { 4.996, 3875.0 },
            { 4.816, 3825.0 },
            { 4.616, 3800.0 },
            { 4.446, 3720.0 },
            { 4.276, 3705.0 },
            { 4.086, 3595.0 },
            { 3.936, 3600.0 },
            { 3.756, 3550.0 },
            { 3.587, 3500.0 },
            { 3.428, 3490.0 },
            { 3.244, 3490.0 },
            { 3.109, 3350.0 },
            { 2.986, 3300.0 },
            { 2.892, 3300.0 },
            { 2.795, 3300.0 },
            { 2.691, 3200.0 },
            { 2.551, 3200.0 },
            { 2.468, 3100.0 },
            { 2.374, 3100.0 },
            { 2.278, 3070.0 },
            { 2.184, 3000.0 },
            { 2.090, 3000.0 },
            { 2.014, 3000.0 },
            { 1.887, 3000.0 },
            { 1.783, 2900.0 },
            { 1.697, 2900.0 },
            { 1.591, 2900.0 },
            { 1.514, 2900.0 },
            { 1.344, 2850.0 },
            { 1.2, 3000.0 },
            { 1, 3000.0 },
    };

    private static double lastFeedingResult = 0;
    private static ShootingParameters currentShootingParameters = new ShootingParameters(0, 0);
    private static InterpolationTable hoodTable = new InterpolationTable(hoodTableData);
    private static InterpolationTable shooterTable = new InterpolationTable(shooterTableData);
    private static boolean automatic = true;
    private static boolean defence = false;
    private static ShootingPreset currentShootingPreset = ShootingPreset.CLOSE;
    public static boolean isLocked = false;
    public static boolean atPointLatch = false;
    public static double startShootingTime = 0;
    public static int ballsShot = 0;
    private static boolean ballLock = false;
    private static double setPointAlign = 0;

    private static Debouncer inTheAirDebouncer = new Debouncer(0.8);
    private static Debouncer sandwichStuckDebouncer = new Debouncer(0.6);
    private static SlewRateLimiter distanceFilter = new SlewRateLimiter(0.1);

    private static double distance = 0;

    public enum ShootingPreset {
        CLOSE(0, 3800, new Pose2d()),
        CLIMB(15.5, 3200, new Pose2d()),
        TRENCH(17.5, 3150, new Pose2d());

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

    public static double getSetPointAlign() {
        return setPointAlign;
    }

    public static void setSetPointAlign(double setPoiint) {
        setPointAlign = setPoiint;
    }

    public static double getAbsAngleToTarget() {

        double xDis = Field.getHub().getX() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getX();
        double yDis = Field.getHub().getY() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getY();
        double angle = Math.atan2(yDis, xDis);

        return Math.toDegrees(angle);

    }

    public static double getAngleToFeeding() {
        double xDis = FEEDING_POSE.getX() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getX();
        double yDis = FEEDING_POSE.getY() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getY();
        double angle = Math.atan2(yDis, xDis);

        return Math.toDegrees(angle);
    }

    public static boolean isHittingNet() {
        return GeometryUtil.willShotHitNet(PoseEstimator.getCurrentPose(), ShooterConstants.SHOOTER_OFFSET,
                Field.getNetA(),
                Field.getNetB(),
                20, FEEDING_IN_MOTION_NET_MARGIN);
    }

    // public static boolean outSideField() {
    // return lastFeedingResult.valid &&
    // Field.FIELD_RECTANGLE.contains(lastFeedingResult.hitPointField)
    // && lastFeedingResult.distanceMeters < FEEDING_IN_MOTION_MIN_DISTANCE;
    // }

    public static boolean isFull() {
        return false;
    }

    public static boolean isBalls() {
        return true;
    }

    public static boolean isExtendedMagazine() {
        return isExtendedMagazine;
    }

    public static void SetIsExtendedMagazine(boolean bool) {
        isExtendedMagazine = bool;
    }

    public static boolean isBallsInSandwich() {
        return true;
        // Sandwich.getInstance().getLeftIr() 
        // ||
        // Sandwich.getInstance().getMiddleIr() ;
        // || Sandwich.getInstance().getRightIr();
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
        return inTheAirDebouncer.calculate(Climb.getInstance().getCurrent() > IN_THE_AIR_CURRENT_THRESHOLD)
                && Climb.getInstance().getPosition() < CLIMB_POSITION_THRESHOLD && Climb.getInstance().isOnBar();
    }

    public static StuckType isStuck() {
        // if (Sandwich.getInstance().isMoving() && isBallsInSandwich()
        // && sandwichStuckDebouncer
        // .calculate(Sandwich.getInstance().getDeltaMAcamDistance() <
        // SANDWICH_STUCK_DELTA)) {
        // return StuckType.STUCK_IN_SANDWICH;
        // } else if (!isBallsInSandwich() && Roller.getInstance().isMoving() &&
        // Transfer.getInstance().isMoving()
        // && isBalls()) {
        // return StuckType.STUCK_IN_TRANSFER;
        // } else {
        return StuckType.NONE;
        // }
    }

    private static double getShootingRPM(double x) {

        if (SwerveController.isAbs < 3) {// Relativ
            return shooterTable.interpolate(x) - 60 > 6000 ? 0 : shooterTable.interpolate(x) - 30 * shootingFactor; // - 60;
        }

        return shooterTable.interpolate(x) > 6000 ? 0 : shooterTable.interpolate(x) - 30 * shootingFactor; // - 60;
    }

    private static double getHoodAngle(double distance) {

        if (hoodTable.interpolate(distance) < 6 && (RobotContainer.getRobotState() == RobotConstants.FEEDING
                || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION)) {
            return 6;
        }

        return hoodTable.interpolate(distance) ;//-2

    }

    public static ShootingParameters getShootingParameters() {
        return currentShootingParameters;
    }

    public static ShootingParameters getFeedingParameters() {
        return new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()) + FEEDING_SHOOTER_OFFSET,
                getHoodAngle(getDistanceToTargetFeeding()) + FEEDING_ANGLE_OFFSET);
    }

    private static double getRelativDistanceToHub() {
        // distance = Math.sqrt(Math.pow(Vision.getInstance().getDistanceTryg(), 2) +
        // Math.pow(Field.HUB_WIDTH / 2, 2)
        // + (2 * Vision.getInstance().getDistanceTryg() * (Field.HUB_WIDTH / 2) *
        // Math.cos(Math.toRadians(
        // VisionConstants.FRONT_LL.getCameraIO().getTag().txnc
        // + Swerve.getInstance().getGyroData().yaw))));

        // return (4.46 * Math.pow(10, -3) + -0.0138 * distance + 0.0383 *
        // Math.pow(distance, 2)) + distance;

        if (SuperStructure.isMainTag()) {
            return Vision.getInstance().getDistanceTryg() + Field.getDistanceToCenterHub(Vision.getInstance().getTagID());
        }
        return Vision.getInstance().getDistanceTryg() ;
    }

    public static double getAbsDistanceToHub() {
        return GeometryUtil
        .poseAdjust(PoseEstimator.getCurrentPose(),
        VisionConstants.FRONTLL_OFFSET)
        .getDistance(Field.getHub());
    }

    private static double getDistanceToTargetShooting() {
        if (!(SwerveController.isAbs < 3) && VisionConstants.FRONT_LL.getCameraIO().isTag()) {

            return getRelativDistanceToHub();
        }

        

        return getAbsDistanceToHub();

    }

    private static double getDistanceToTargetFeeding() {
        if (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            lastFeedingResult = distanceToMotionFeeding(PoseEstimator.getCurrentPose().getX() - Field.getFeedingLine(),
                    Swerve.getInstance().getGyroYawSupplier().get());
            MALog.log("/Superstructure/Feeding In Motion Distance", lastFeedingResult);
            MALog.log("/Superstructure/X Line", Field.getFeedingLine());
            return lastFeedingResult;
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING) {
            return GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(), VisionConstants.FRONTLL_OFFSET)
                    .getDistance(FEEDING_POSE);
        }

        return -1;
    }

    public static Supplier<Double> getGyroSUpplierFOrRelativAlign() {
        if (Vision.getInstance().getTagID() == 21 || Vision.getInstance().getTagID() == 5) {
            return () -> Swerve.getInstance().getGyroYawSupplier().get() + 90;
        } else if (Vision.getInstance().getTagID() == 18 || Vision.getInstance().getTagID() == 2) {
            return () -> Swerve.getInstance().getGyroYawSupplier().get() - 90;
        } else {
            return Swerve.getInstance().getGyroYawSupplier();
        }
    }

    public static boolean isInTheAlinceZone() {
        return DriverStationUtil.getAlliance() == Alliance.Blue
                ? PoseEstimator.getCurrentPose().getX() < Field.ALLIANCE_WIDTH
                : PoseEstimator.getCurrentPose().getX() > Field.LENGTH - Field.ALLIANCE_WIDTH;
    }

    public static boolean isInWarmUpZone() {
        return DriverStationUtil.getAlliance() == Alliance.Blue
                ? PoseEstimator.getCurrentPose().getX() < Field.ALLIANCE_WIDTH + 2
                : PoseEstimator.getCurrentPose().getX() > Field.LENGTH - Field.ALLIANCE_WIDTH - 2;
    }

    public static boolean atPointForShooting() {

        return (atPointLatch || Shooter.getInstance().atPointForShooting()) 
                && Hood.getInstance().atPointForShooting() &&  (SwerveConstants.ANGLE_ADJUST_CONTROLLER.atSetpoint() || (!isAutomatic() ));

    }

    public static boolean atPointForFeeding() {

        return (atPointLatch || Shooter.getInstance().atPointForFeeding())
                && Hood.getInstance().atPointForFeeding()
                && !isHittingNet();
    }

    public static boolean atPointForFeedingInMotion() {

        return (atPointLatch || Shooter.getInstance().atPointForShooting()) && isOkAngleForFeeding()
                && Hood.getInstance().atPointForFeedingInMotion() && !isHittingNet()
                && distanceYToMotionFeeding(Field.getFeedingLine(), Swerve.getInstance().getGyroYawSupplier().get());
    }

    public static boolean isOkAngleForFeeding() {
        if ((PoseEstimator.getCurrentPose().getRotation().getDegrees() > 0
                && PoseEstimator.getCurrentPose().getRotation().getDegrees() > 135
                || PoseEstimator.getCurrentPose().getRotation().getDegrees() < 0
                        && PoseEstimator.getCurrentPose().getRotation().getDegrees() < -135) && DriverStationUtil.getAlliance() == Alliance.Blue) {
            return true;
        }

        if (PoseEstimator.getCurrentPose().getRotation().getDegrees() < 45
                        && PoseEstimator.getCurrentPose().getRotation().getDegrees() > -45  && DriverStationUtil.getAlliance() == Alliance.Red) {
                            return true;
                        }

        return false;
    }

    public static ShootingPreset getCurrentShootingPreset() {
        return currentShootingPreset;
    }

    public static void setCurrentShootingPreset(ShootingPreset preset) {
        currentShootingPreset = preset;
    }

    public static double distanceToMotionFeeding(double Xline, double relativGyro) {
        return Math.abs(Xline * Math.cos(Math.toRadians(relativGyro - 180)));
    }

    public static boolean distanceYToMotionFeeding(double Xline, double relativGyro) {
        MALog.log("/SuperStructure/Y Distance", Math.abs(Xline * Math.sin(Math.toRadians(relativGyro - 180))));
        if (PoseEstimator.getCurrentPose().getRotation().getDegrees() > 0) {
            return PoseEstimator.getCurrentPose().getY()
                    + Math.abs(Xline * Math.sin(Math.toRadians(relativGyro - 180))) < (Field.WIDTH
                            - FEEDING_IN_MOTION_FIELD_MARGIN);
        }

        return PoseEstimator.getCurrentPose().getY()
                - Math.abs(Xline * Math.sin(Math.toRadians(relativGyro - 180))) > (0
                        + FEEDING_IN_MOTION_FIELD_MARGIN);
    }

    public static double getTransferSinVoltage() {
        double f = 1;
        return ((Math.cos(2 * Math.PI * f * (Timer.getFPGATimestamp() - startShootingTime))) * 1.5 + 8.5);
    }

    public static double getRollerSinVoltage() {
        double f = 1;
        return ((Math.cos(2 * Math.PI * f * (Timer.getFPGATimestamp() - startShootingTime))) * 0.75 + 8.5);
    }

    public static double getSetPointForShootingRel(double id) {
        if (id == 27 ||id == 11) {
            return 2;
        } else if (id == 24 || id == 8) {
            return -2;
        } else if ((id == 10 || id == 26) && PoseEstimator.getCurrentPose().getY() > Field.WIDTH / 2) {
            return -5;
        } else if ((id == 10 || id == 26) && PoseEstimator.getCurrentPose().getY() < Field.WIDTH / 2) {
            return 5;
        } 

        return 0;
    }

    public void setShooterFactor(double newFactor) {
        shootingFactor = newFactor;
    }

    public double getShooterFactor() {
        return shootingFactor;
    }

    public static void update() {
        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && !isLocked) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetShooting()),
                    getHoodAngle(getDistanceToTargetShooting()));
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING
                || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            currentShootingParameters = new ShootingParameters(
                    getShootingRPM(getDistanceToTargetFeeding() + FEEDING_DISTANCE_OFFSET),
                    getHoodAngle(getDistanceToTargetFeeding() + FEEDING_DISTANCE_OFFSET));
        }

        if (SwerveConstants.ANGLE_ADJUST_CONTROLLER.atSetpoint() && RobotContainer.getRobotState() != RobotConstants.SHOOTING_UNLOCKED) {
            isLocked = true;
        }

        if (Shooter.getInstance().atPoint()) {
            atPointLatch = true;
        }

        MALog.log("/SuperStructure/Shooting Factor", (shootingFactor - 1) * 100);
        MALog.log("/SuperStructure/Angle Abs", getAbsAngleToTarget());
        MALog.log("/SuperStructure/Distance/Abs", getAbsDistanceToHub());
        MALog.log("/SuperStructure/Distance/Shooter Pose", GeometryUtil
        .poseAdjust(PoseEstimator.getCurrentPose(),
        VisionConstants.FRONTLL_OFFSET)
        .getDistance(Field.getHub()));
        MALog.log("/SuperStructure/Distance/Trigo Distance Tag", getRelativDistanceToHub());
        MALog.log("/SuperStructure/Shooter Velo", currentShootingParameters.shooterRPM());
        MALog.log("/SuperStructure/Hood Angle", currentShootingParameters.hoodAngle());
        MALog.log("/SuperStructure/Shooting Latch", atPointLatch);

        MALog.log("/SuperStructure/Offset Pose",
                new Pose2d(GeometryUtil.poseAdjust(
                        PoseEstimator.getCurrentPose(),
                        VisionConstants.FRONTLL_OFFSET), new Rotation2d()));
        MALog.log("/SuperStructure/Is in alliance zone", isInTheAlinceZone());

        MALog.log("/SuperStructure/At Point For Shooting", atPointForShooting());
        MALog.log("/SuperStructure/Hub Pose", Field.getHub());

        MALog.log("/SuperStructure/Feeding Pose", FEEDING_POSE);
        MALog.log("/SuperStructure/Feeding Distance", getDistanceToTargetFeeding());
        MALog.log("/SuperStructure/Is Net", isHittingNet());
        MALog.log("/SuperStructure/Is at point - angle adjust controller",
                SwerveConstants.ANGLE_ADJUST_CONTROLLER.atSetpoint());
        MALog.log("/SuperStructure/Is delta tx", Vision.getInstance().isDeltaTx());

        MALog.log("/SuperStructure/Is Y In Field",
                distanceYToMotionFeeding(Field.getFeedingLine(), Swerve.getInstance().getGyroYawSupplier().get()));

        // if (ballsShot == 0 && Shooter.getInstance().getSensor() && !isLocked) {
        // startShootingTime = Timer.getFPGATimestamp();
        // }

        // if (!isLocked && Shooter.getInstance().getSensor()) {
        // ballsShot++;
        // isLocked = true;
        // }

        // if (!Shooter.getInstance().getSensor()) {
        // isLocked = false;
        // }

        MALog.log("/SuperStructure/Aavrage BPS", ballsShot / (Timer.getFPGATimestamp() - startShootingTime));
        MALog.log("/SuperStructure/Num Balls", ballsShot);
        MALog.log("/SuperStructure/Sin Voltage", getTransferSinVoltage());

        MALog.log("/SuperStructure/Time left in teleop", DriverStation.getMatchTime());

        MALog.log("/SuperStructure/is abs", SwerveController.isAbs);
        MALog.log("/SuperStructure/Is In Warmup", isInWarmUpZone());
        MALog.log("/SuperStructure/Shooter at point for shooting", atPointLatch || Shooter.getInstance().atPointForShooting());
        MALog.log("/SuperStructure/Shooter at point for feeding",  (atPointLatch || Shooter.getInstance().atPointForShooting()));
        MALog.log("/SuperStructure/ok feeding angle", isOkAngleForFeeding() );
        MALog.log("/SuperStructure/hitting net?",  !isHittingNet() );



        MALog.log("/SuperStructure/swerve at point for shooting",  SwerveConstants.ANGLE_ADJUST_CONTROLLER.atSetpoint() || (!isAutomatic() ));
        MALog.log("/SuperStructure/swerve at point for feeding", distanceYToMotionFeeding(Field.getFeedingLine(), Swerve.getInstance().getGyroYawSupplier().get()));


    }

}
