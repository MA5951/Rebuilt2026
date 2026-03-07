package com.MAutils.PoseEstimation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Vision.Filters.FiltersConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Util.Field;

/**
 * PoseEstimator holds any number of PoseEstimatorSource,
 * fuses them each loop by XY/θ FOM-weighted averages, and corrects for latency
 * by replaying a short history when late measurements arrive.
 */
public class PoseEstimator {
  // private static final double HISTORY_WINDOW_SEC = 0.5; // must exceed max
  // latency

  // // Numerical guard for FOM sums
  // private static final double FOM_EPS = 1e-9;

  // /**
  // * Motion sanity limits (used to clamp per-step twist when integrating).
  // * Consider wiring these to drivetrain constants.
  // */
  // private static final double MAX_TRANSLATION_VEL_MPS = 8.0; // typical fast
  // swerve ballpark
  // private static final double MAX_ANGULAR_VEL_RADPS = 14.0; // ~800 deg/s
  // ballpark

  // private static final List<PoseEstimatorSource> sources = new ArrayList<>();

  // private static SwerveDriveSimulation swerveSim = null;

  // // Pose and time just before our replay buffer begins
  // private static Pose2d poseBeforeHistory = new Pose2d();
  // private static double historyStartTime;

  // // Recent applied twists, for replay
  // private static final Deque<HistoryEntry> history = new ArrayDeque<>();

  // private static Pose2d currentPose = new Pose2d();
  // private static double lastUpdateTime;

  // /**
  // * Optional hook: if a source supports pruning, we can ask it to drop
  // * measurements older than our history window.
  // */
  // public interface PrunablePoseEstimatorSource {
  // void pruneBefore(double timestamp);
  // }

  // public static void setSwerveSim(SwerveDriveSimulation sim) {
  // swerveSim = sim;
  // }

  // public static void resetPose(Pose2d newPose) {
  // double now = Timer.getFPGATimestamp();
  // history.clear();
  // poseBeforeHistory = newPose;
  // historyStartTime = now;
  // currentPose = newPose;
  // lastUpdateTime = now;

  // TelemetryLogger.logPoseEstimator(
  // "Pose reset - X:" + newPose.getX()
  // + " Y:" + newPose.getY()
  // + " R:" + newPose.getRotation().getDegrees());

  // if (!Robot.isReal() && swerveSim != null) {
  // swerveSim.setSimulationWorldPose(newPose);
  // }
  // }

  // /** Add odometry / limelight / any other source */
  // public static void addSource(PoseEstimatorSource src) {
  // TelemetryLogger.logPoseEstimator("Added source: " + src.name);
  // sources.add(src);
  // }

  // /** Call once per robot loop. */
  // public static void update() {
  // final double now = Timer.getFPGATimestamp();

  // // Ask sources to prune anything older than our history window (optional).
  // pruneSources(now - HISTORY_WINDOW_SEC - 0.05); // small cushion

  // // If any source has a measurement stamped before our last update, we need to
  // replay.
  // // NOTE: This assumes sources only report "unconsumed" late packets via
  // hasBefore().
  // // If a source keeps old packets forever and hasBefore() doesn't account for
  // consumption,
  // // it should be fixed inside that source (or implement
  // PrunablePoseEstimatorSource).
  // if (hasLatePackets()) {
  // replayHistory();
  // }

  // // Apply one step at 'now'
  // applyAtTime(now, history, true);

  // // Trim old history
  // trimHistory();
  // }

  // public static Pose2d getCurrentPose() {
  // if (currentPose == null) {
  // return new Pose2d(-1, -1, new Rotation2d());
  // }
  // return currentPose;
  // }

  // /** Backward-compatible overall FOM (mean of XY and θ across sources). */
  // public static double getRobotFOM() {
  // return sources.stream()
  // .mapToDouble(s -> s.getFomAt(lastUpdateTime))
  // .average()
  // .orElse(0.0);
  // }

  // /** New: average XY FOM across sources at the last update. */
  // public static double getRobotFOMXY() {
  // return sources.stream()
  // .mapToDouble(s -> s.getFomXYAt(lastUpdateTime))
  // .average()
  // .orElse(0.0);
  // }

  // public static Pose2d getPoseLookAhead(double time, ChassisSpeeds
  // speedsRobotRelativ) {
  // return currentPose.exp(
  // new Twist2d(
  // speedsRobotRelativ.vxMetersPerSecond * time,
  // speedsRobotRelativ.vyMetersPerSecond * time,
  // speedsRobotRelativ.omegaRadiansPerSecond * time));
  // }

  // // —— internal —— //

  // /** Pose reconstructed by applying history up to a query time. */
  // public static Pose2d getPoseAt(double queryTime) {
  // Pose2d pose = poseBeforeHistory;
  // for (HistoryEntry e : history) {
  // if (e.time > queryTime) break;
  // pose = pose.exp(e.twist);
  // }
  // return pose;
  // }

  // /** True if any source claims it has a packet older than lastUpdateTime
  // (late/out-of-order). */
  // private static boolean hasLatePackets() {
  // return sources.stream().anyMatch(s -> s.hasBefore(lastUpdateTime));
  // }

  // private static void pruneSources(double pruneBeforeTime) {
  // for (PoseEstimatorSource src : sources) {
  // if (src instanceof PrunablePoseEstimatorSource) {
  // ((PrunablePoseEstimatorSource) src).pruneBefore(pruneBeforeTime);
  // }
  // }
  // }

  // /**
  // * Replay from poseBeforeHistory through all history entries, slotting in any
  // late packets.
  // */
  // private static void replayHistory() {
  // // We assume 'history' timestamps are monotonic-increasing. If that invariant
  // is ever broken,
  // // replay should sort (but we avoid allocations/sorts unless needed).
  // Pose2d pose = poseBeforeHistory;
  // Deque<HistoryEntry> newHist = new ArrayDeque<>();

  // // Rebuild by re-computing fused twists at each stored history timestamp
  // for (HistoryEntry old : history) {
  // // Temporarily set currentPose so applyAtTime() uses correct base pose while
  // rebuilding
  // currentPose = pose;
  // applyAtTime(old.time, newHist, false);
  // pose = currentPose;
  // }

  // // Validate reconstructed pose
  // if (Field.ALLOWED_FIELD.contains(pose.getTranslation()) &&
  // !Field.HUB_BLUE.contains(pose.getTranslation()) &&
  // !Field.HUB_RED.contains(pose.getTranslation())) {
  // history.clear();
  // history.addAll(newHist);
  // currentPose = pose;

  // HistoryEntry last = history.peekLast();
  // lastUpdateTime = (last == null) ? historyStartTime : last.time;

  // // Keep window sane after replay
  // trimHistory();
  // } else {
  // TelemetryLogger.logPoseEstimator(
  // "Update rejected in replay because pose is outside the field");
  // }
  // }

  // /**
  // * Applies the fused twist at timestamp t onto currentPose.
  // *
  // * @param timestamp integration time
  // * @param targetHist where to write the history entry (can be a temp deque
  // during replay)
  // * @param logNowPose whether to log the final pose
  // */
  // private static void applyAtTime(double timestamp, Deque<HistoryEntry>
  // targetHist, boolean logNowPose) {
  // final double dt = Math.max(0.0, timestamp - lastUpdateTime);

  // Twist2d fused = computeFusedTwist(timestamp);

  // // Sanity clamp per-step motion to something physically plausible
  // fused = clampTwistByDt(fused, dt);

  // Pose2d candidate = currentPose.exp(fused);

  // if (Field.ALLOWED_FIELD.contains(candidate.getTranslation()) &&
  // !Field.HUB_BLUE.contains(candidate.getTranslation()) &&
  // !Field.HUB_RED.contains(candidate.getTranslation())) {
  // currentPose = candidate;
  // targetHist.addLast(new HistoryEntry(timestamp, fused));
  // lastUpdateTime = timestamp;

  // if (logNowPose) {
  // MALog.log("Pose Estimator/Current Pose", currentPose);
  // }
  // } else {
  // TelemetryLogger.logPoseEstimator("Update rejected because pose is outside the
  // field");
  // }
  // }

  // private static Twist2d clampTwistByDt(Twist2d t, double dt) {
  // if (dt <= 0.0) return t;

  // double maxDx = MAX_TRANSLATION_VEL_MPS * dt;
  // double maxDy = MAX_TRANSLATION_VEL_MPS * dt;
  // double maxDTheta = MAX_ANGULAR_VEL_RADPS * dt;

  // double dx = clamp(t.dx, -maxDx, maxDx);
  // double dy = clamp(t.dy, -maxDy, maxDy);
  // double dtheta = clamp(t.dtheta, -maxDTheta, maxDTheta);

  // return new Twist2d(dx, dy, dtheta);
  // }

  // private static double clamp(double v, double lo, double hi) {
  // return Math.max(lo, Math.min(hi, v));
  // }

  // /**
  // * Weighted average of each source’s twist at exactly timestamp T
  // * (translation weighted by XY FOM; rotation weighted by θ FOM).
  // */
  // private static Twist2d computeFusedTwist(double timestamp) {
  // double sumFomXY = 0.0;
  // double sumFomTheta = 0.0;

  // double dxAcc = 0.0;
  // double dyAcc = 0.0;
  // double dThetaAcc = 0.0;

  // for (PoseEstimatorSource src : sources) {
  // Twist2d tt = src.getTwistAt(timestamp);
  // if (tt == null) continue;

  // double fXY = src.getFomXYAt(timestamp);
  // double fTh = src.getFomThetaAt(timestamp);

  // // Defensive: ignore negative/NaN FOMs
  // if (!(fXY > 0.0)) fXY = 0.0;
  // if (!(fTh > 0.0)) fTh = 0.0;

  // dxAcc += tt.dx * fXY;
  // dyAcc += tt.dy * fXY;
  // dThetaAcc += tt.dtheta * fTh;

  // sumFomXY += fXY;
  // sumFomTheta += fTh;
  // }

  // // Avoid division by ~0; allow translation-only or rotation-only fusion
  // final boolean hasXY = sumFomXY > FOM_EPS;
  // final boolean hasTh = sumFomTheta > FOM_EPS;

  // if (!hasXY && !hasTh) {
  // return new Twist2d();
  // }

  // double outDx = hasXY ? (dxAcc / sumFomXY) : 0.0;
  // double outDy = hasXY ? (dyAcc / sumFomXY) : 0.0;
  // double outDTh = hasTh ? (dThetaAcc / sumFomTheta) : 0.0;

  // return new Twist2d(outDx, outDy, outDTh);
  // }

  // /** Drop anything older than our window and roll poseBeforeHistory forward.
  // */
  // private static void trimHistory() {
  // double cutoff = lastUpdateTime - HISTORY_WINDOW_SEC;

  // while (!history.isEmpty() && history.peekFirst().time < cutoff) {
  // HistoryEntry e = history.removeFirst(); // explicit type (more readable here)
  // poseBeforeHistory = poseBeforeHistory.exp(e.twist);
  // historyStartTime = e.time;
  // }
  // }

  // private static class HistoryEntry {
  // final double time;
  // final Twist2d twist;

  // HistoryEntry(double t, Twist2d tw) {
  // time = t;
  // twist = tw;
  // }
  // }

  public record OdometryObservation(
      double timestamp,
      SwerveModulePosition[] startWheelPositions,
      Optional<Rotation2d> roll,
      Optional<Rotation2d> pitch,
      Optional<Rotation2d> yaw) {
  }

  public record VisionObservation(double timestamp, Pose2d visionPose, Matrix<N3, N1> stdDevs) {
  }

  public static SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private static final double poseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  private static Pose2d odometryPose = Pose2d.kZero;
  private static Pose2d estimatedPose = Pose2d.kZero;

  private static final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
      .createBuffer(poseBufferSizeSec);
  private static final TimeInterpolatableBuffer<Rotation3d> rotationBuffer = TimeInterpolatableBuffer
      .createBuffer(poseBufferSizeSec);
  private static final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private static Rotation2d gyroOffset = Rotation2d.kZero;

  public static void init() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }

    // lastWheelPositions = Swerve.getInstance().getCurrentPositions();
  }

  public static void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated -
    // offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public static Pose2d getCurrentPose() {
    return estimatedPose;
  }

  public static void update() {
    MALog.log("Pose Estimator/Current Pose", getCurrentPose());
  }

  public static void addOdometryObservation(OdometryObservation observation) {
    // Update odometry pose
    Twist2d twist = SwerveConstants.SWERVE_CONSTANTS.kinematics.toTwist2d(lastWheelPositions,
        observation.startWheelPositions());

    lastWheelPositions = observation.startWheelPositions().clone();

    Pose2d lastOdometryPose = odometryPose;
    MALog.log("Pose Estimator/Last Odometry", lastOdometryPose);
    odometryPose = odometryPose.exp(twist);
    MALog.log("Pose Estimator/Updated Odometry", odometryPose);

    // Replace odometry pose with gyro if present
    observation.yaw.ifPresent(
        gyroAngle -> {
          // Add offset to measured angle
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    // Add rotation to buffer at timestamp
    if (observation.roll.isPresent()) {
      rotationBuffer.addSample(
          observation.timestamp(),
          new Rotation3d(
              observation.roll.get().getRadians(),
              observation.pitch.get().getRadians(),
              observation.yaw.get().getRadians()));
    }
    // Apply odometry delta to vision pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    Pose2d tempPose1 =  estimatedPose.exp(finalTwist);

    if (validatePose(tempPose1)) {
      estimatedPose = tempPose1;
      MALog.log("Pose Estimator/Odometry Status", "Applied odometry pose estimate");
    } else {
      MALog.log("Pose Estimator/Odometry Status", "Invalid odometry pose estimate received; ignoring.");
    }
  }

  /** Adds a new vision pose observation from the vision subsystem. */
  public static void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // Calculate transforms between odometry pose and vision sample pose
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

    // Shift estimated pose backwards to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Calculate the transform from the shifted estimate to the observation pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());

    // Scale the transform by the Kalman gain
    var kTimesTransform = visionK.times(
        VecBuilder.fill(
            transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform = new Transform2d(
        kTimesTransform.get(0, 0),
        kTimesTransform.get(1, 0),
        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate the current estimate by applying the scaled transform to the old
    // estimate
    // then shifting forwards using odometry data
    Pose2d tempPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    if (validatePose(tempPose)) {
      estimatedPose = tempPose;
      MALog.log("Pose Estimator/Vision Status", "Applied vision pose estimate with Kalman gain");
      
    } else {
      MALog.log("Pose Estimator/Vision Status", "Invalid vision pose estimate received; ignoring.");
    }
    
    
  }

  public static boolean validatePose(Pose2d pose) {
    return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians()) && !Double.isInfinite(pose.getX()) && !Double.isInfinite(pose.getY()) && !Double.isInfinite(pose.getRotation().getRadians());
  }
}
