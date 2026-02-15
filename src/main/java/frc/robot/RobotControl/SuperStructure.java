
package frc.robot.RobotControl;

import static edu.wpi.first.units.Units.Meters;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultSuperStructure;
import com.MAutils.Utils.ChassisSpeedsUtil;
import com.MAutils.Utils.DriverStationUtil;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Commands.SwerveController;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Roller.Roller;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Util.BooleanLatch;
import frc.robot.Util.Field;
import frc.robot.Util.GeometryUtil;
import frc.robot.Util.ShootingParameters;
import frc.robot.Util.GeometryUtil.Result;
import frc.robot.Util.InterpolationTable;

public class SuperStructure extends DeafultSuperStructure {
    public static final Translation2d FEEDING_POSE = new Translation2d(2, 2);

    public static final double IN_THE_AIR_CURRENT_THRESHOLD = 40.0;
    public static final double CLIMB_POSITION_THRESHOLD = 0.15;

    public static final double FEEDING_ANGLE_OFFSET = 5;
    public static final double FEEDING_SHOOTER_OFFSET = 300;

    public static final double FEEDING_IN_MOTION_FIELD_MARGIN = 1;
    public static final double FEEDING_IN_MOTION_NET_MARGIN = 0.5;
    public static final double FEEDING_IN_MOTION_MIN_DISTANCE = 1;

    public static final double SANDWICH_STUCK_DELTA = 10;

    public static double AFTER_ANGLE = 0;

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
            { 3.587, 22.0 },
            { 3.428, 21.0 },
            { 3.244, 20.0 },
            { 3.109, 20.0 },
            { 2.986, 19.0 },
            { 2.892, 18.0 },
            { 2.795, 17.0 },
            { 2.691, 16.0 },
            { 2.551, 15.5 },
            { 2.468, 15.0 },
            { 2.374, 14.0 },
            { 2.278, 13.0 },
            { 2.184, 13.0 },
            { 2.090, 12.5 },
            { 2.014, 12.0 },
            { 1.887, 11.0 },
            { 1.783, 10.0 },
            { 1.697, 9.0 },
            { 1.591, 8.0 },
            { 1.514, 7.0 },
            { 1.344, 6.0 },
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
            { 3.756, 3600.0 },
            { 3.587, 3550.0 },
            { 3.428, 3550.0 },
            { 3.244, 3500.0 },
            { 3.109, 3250.0 },
            { 2.986, 3200.0 },
            { 2.892, 3200.0 },
            { 2.795, 3200.0 },
            { 2.691, 3200.0 },
            { 2.551, 3200.0 },
            { 2.468, 3100.0 },
            { 2.374, 3075.0 },
            { 2.278, 3050.0 },
            { 2.184, 3000.0 },
            { 2.090, 3000.0 },
            { 2.014, 2950.0 },
            { 1.887, 2750.0 },
            { 1.783, 2750.0 },
            { 1.697, 2700.0 },
            { 1.591, 2700.0 },
            { 1.514, 2700.0 },
            { 1.344, 2650.0 },
    };

    private static Result lastFeedingResult = new Result(false, -1, new Translation2d());
    private static ShootingParameters currentShootingParameters = new ShootingParameters(0, 0);
    private static InterpolationTable hoodTable = new InterpolationTable(hoodTableData);
    private static InterpolationTable shooterTable = new InterpolationTable(shooterTableData);
    private static boolean automatic = true;
    private static boolean defence = false;
    private static ShootingPreset currentShootingPreset = ShootingPreset.CLOSE;
    public static boolean isLocked = false;
    public static boolean atPointLatch = false;

    private static Debouncer inTheAirDebouncer = new Debouncer(0.8);
    private static Debouncer sandwichStuckDebouncer = new Debouncer(0.6);
    private static SlewRateLimiter distanceFilter = new SlewRateLimiter(0.1);

    private static double distance = 0;

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
        return 90 + Math.toDegrees(Math.atan2(
                Vision.getInstance().getDistanceTryg()
                        * Math.sin(Math.toRadians(90 - VisionConstants.FRONT_LL.getCameraIO().getTag().txnc
                                - Swerve.getInstance().getGyroData().yaw + 90))
                        + Field.HUB_WIDTH / 2,
                Vision.getInstance().getDistanceTryg()
                        * Math.cos(Math.toRadians(90 - VisionConstants.FRONT_LL.getCameraIO().getTag().txnc
                                - Swerve.getInstance().getGyroData().yaw + 90))));
    }

    public static double getAbsAngleToTarget() {

        double xDis = Field.getHub().getX() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getX();
        double yDis = Field.getHub().getY() - GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
                VisionConstants.FRONTLL_OFFSET).getY();
        double angle = Math.atan2(yDis, xDis);

        return Math.toDegrees(angle);
        // return GeometryUtil.angleTo(new
        // Pose2d(GeometryUtil.poseAdjust(PoseEstimator.getCurrentPose(),
        // VisionConstants.FRONTLL_OFFSET),
        // PoseEstimator.getCurrentPose().getRotation()), Field.getHub());
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
                && lastFeedingResult.distanceMeters < FEEDING_IN_MOTION_MIN_DISTANCE;
    }

    public static boolean isFull() {
        return false;
    }

    public static boolean isBalls() {
        return true;
    }

    public static boolean isBallsInSandwich() {
        // return Sandwich.getInstance().getMACamDistance() <
        // SandwichConstants.IS_BALLS_DISTANCE
        // || Sandwich.getInstance().getEndSensor();
        return true;
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
        // return (2761
        // -290* x
        // + 192 * Math.pow(x, 2)
        // - 17.4 * Math.pow(x, 3)
        // ) + 175;

        // 2761 + -309x + 192x^2 + -17.4x^3

        if (SwerveController.isAbs) {
                return shooterTable.interpolate(x) - 60 > 6000 ? 0 : shooterTable.interpolate(x) - 60;
        }

        return shooterTable.interpolate(x) > 6000 ? 0 : shooterTable.interpolate(x);
    }

    private static double getHoodAngle(double distance) {
        // return -7.75
        // + 11.6 * distance
        // - 0.989 * Math.pow(distance, 2);

        return hoodTable.interpolate(distance);
    }// -7.75 + 11.6x + -0.989x^2

    public static ShootingParameters getShootingParameters() {
        return currentShootingParameters;
    }

    public static ShootingParameters getFeedingParameters() {
        return new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()) + FEEDING_SHOOTER_OFFSET,
                getHoodAngle(getDistanceToTargetFeeding()) + FEEDING_ANGLE_OFFSET);
    }

    private static double getDistanceToTargetShooting() {
        if (Vision.getInstance().isMainTag() && !SwerveController.isAbs) {
            distance = Math.sqrt(Math.pow(Vision.getInstance().getDistanceTryg(), 2) +
                    Math.pow(Field.HUB_WIDTH / 2, 2)
                    + (2 * Vision.getInstance().getDistanceTryg() * (Field.HUB_WIDTH / 2) *
                            Math.cos(Math.toRadians(
                                    VisionConstants.FRONT_LL.getCameraIO().getTag().txnc
                                            + Swerve.getInstance().getGyroData().yaw))));

            return (4.46 * Math.pow(10, -3) + -0.0138 * distance + 0.0383 *
                    Math.pow(distance, 2)) + distance;
        }

        return GeometryUtil
                .poseAdjust(PoseEstimator.getCurrentPose(),
                        VisionConstants.FRONTLL_OFFSET)
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

    public static double getAFTERANGLE() {
        return AFTER_ANGLE < 110 ? AFTER_ANGLE - 3 : 177 - AFTER_ANGLE;
    }

    public static boolean isInTheAlinceZone() {
        return DriverStationUtil.getAlliance() == Alliance.Blue
                ? PoseEstimator.getCurrentPose().getX() < Field.ALLIANCE_WIDTH
                : PoseEstimator.getCurrentPose().getX() > Field.LENGTH - Field.ALLIANCE_WIDTH;
    }

    public static boolean atPointForShooting() {
        // return atPointLatch.calculate((Swerve.getInstance().atPointForShooting() ||
        // !isAutomatic()) && Shooter.getInstance().atPointForShooting() &&
        // Hood.getInstance().atPointForShooting()); //Full Latch

        return (atPointLatch || Shooter.getInstance().atPointForShooting())
                && Hood.getInstance().atPointForShooting()
                && (Vision.getInstance().isDeltaTx() || SwerveConstants.REL_PID_CONTROLLER.atSetpoint()
                        || !isAutomatic()); // Intiligent Latch

        // return (Swerve.getInstance().atPointForShooting() ) &&
        // Shooter.getInstance().atPointForShooting() &&
        // Hood.getInstance().atPointForShooting();//No Latch
    }

    public static boolean atPointForFeeding() {
        // return atPointLatch.calculate((Swerve.getInstance().atPointForFeeding() ||
        // !isAutomatic()) && Shooter.getInstance().atPointForFeeding() &&
        // Hood.getInstance().atPointForFeeding()); //Full Latch

        // return atPointLatch.calculate(Shooter.getInstance().atPointForFeeding())
        // && Hood.getInstance().atPointForFeeding()
        // && (Swerve.getInstance().atPointForFeeding() || !isAutomatic()); //
        // Intiligent Latch

        return (Swerve.getInstance().atPointForFeeding()) &&
                Shooter.getInstance().atPointForFeeding() &&
                Hood.getInstance().atPointForFeeding(); // No Latch
    }

    public static boolean atPointForFeedingInMotion() {
        // return
        // atPointLatch.calculate((Swerve.getInstance().atPointForFeedingInMotion() ||
        // !isAutomatic()) && Shooter.getInstance().atPointForFeedingInMotion() &&
        // Hood.getInstance().atPointForFeedingInMotion()); //Full Latch

        // return
        // atPointLatch.calculate(Shooter.getInstance().atPointForFeedingInMotion())
        // && Hood.getInstance().atPointForFeedingInMotion()
        // && (Swerve.getInstance().atPointForFeedingInMotion() || !isAutomatic()); //
        // Intiligent Latch

        return (Swerve.getInstance().atPointForFeedingInMotion() || !isAutomatic())
                && Shooter.getInstance().atPointForFeedingInMotion() &&
                Hood.getInstance().atPointForFeedingInMotion(); // No Latch
    }

    public static ShootingPreset getCurrentShootingPreset() {
        return currentShootingPreset;
    }

    public static void setCurrentShootingPreset(ShootingPreset preset) {
        currentShootingPreset = preset;
    }

    public static void update() {
        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING && !isLocked) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetShooting()),
                    getHoodAngle(getDistanceToTargetShooting()));
        } else if (RobotContainer.getRobotState() == RobotConstants.FEEDING
                || RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            currentShootingParameters = new ShootingParameters(getShootingRPM(getDistanceToTargetFeeding()),
                    getHoodAngle(getDistanceToTargetFeeding()));
        }

        if (Vision.getInstance().getDeltaTX() < 2.5 && SwerveConstants.ANGLE_ADJUST_CONTROLLER.atSetpoint()) {
            isLocked = true;
        }

        if (Shooter.getInstance().atPointForShooting()) {
            atPointLatch = true;
        }

        MALog.log("/SuperStructure/Angle Abs", getAbsAngleToTarget());
        MALog.log("/SuperStructure/TrigoDistance Abs",
                GeometryUtil
                        .poseAdjust(PoseEstimator.getCurrentPose(),
                                VisionConstants.FRONTLL_OFFSET)
                        .getDistance(Field.getHub()));
        MALog.log("/SuperStructure/X Dis", Vision.getInstance().getDistanceTryg());
        MALog.log("/SuperStructure/Shooter Velo", currentShootingParameters.shooterRPM());
        MALog.log("/SuperStructure/Hood Angle", currentShootingParameters.hoodAngle());

        MALog.log("/SuperStructure/Offset Pose",
                new Pose2d(GeometryUtil.poseAdjust(
                        PoseEstimator.getCurrentPose(),
                        VisionConstants.FRONTLL_OFFSET), new Rotation2d()));
        MALog.log("/SuperStructure/Is in alliance zone", isInTheAlinceZone());

        // MALog.log("/SuperStructure/G Left",
        // Swerve.getInstance().getGyroYawSupplier().get()- 90);

        MALog.log("/SuperStructure/G Right", 360 - (90 - Swerve.getInstance().getGyroYawSupplier().get() + 180));

        double totAngle = 360 - (90 - Swerve.getInstance().getGyroYawSupplier().get() + 180
                + (Vision.getInstance().getFilteredTx()));
        double Y = Vision.getInstance().getDistanceTryg() * Math.sin(Math.toRadians(totAngle));
        double X = Vision.getInstance().getDistanceTryg() * Math.cos(Math.toRadians(totAngle));
        double YL = Y + Field.HUB_WIDTH / 2;

        double finAngle = 90 - Math.toDegrees(Math.atan(YL / X));

        MALog.log("/SuperStructure/Fin Angle", finAngle);

        if (Vision.getInstance().getFilteredTx() > 0 && AFTER_ANGLE < 110) {
            MALog.log("/SuperStructure/After Angle", -finAngle);
            AFTER_ANGLE = -finAngle;
        } else {
            MALog.log("/SuperStructure/After Angle", finAngle);
            AFTER_ANGLE = finAngle;
        }

        MALog.log("/SuperStructure/At Point For Shooting", atPointForShooting());
        MALog.log("/SuperStructure/Hub Pose", Field.getHub());

    }

}
