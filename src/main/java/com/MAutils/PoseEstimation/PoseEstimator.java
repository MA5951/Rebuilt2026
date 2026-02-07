package com.MAutils.PoseEstimation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.stream.Collectors;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Vision.Filters.FiltersConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * PoseEstimator holds any number of PoseEstimatorSource,
 * fuses them each loop by XY/θ FOM-weighted averages, and corrects for latency
 * by replaying a short history when late measurements arrive.
 */
public class PoseEstimator {
    private static final double HISTORY_WINDOW_SEC = 0.5; // must exceed max latency

    private static final List<PoseEstimatorSource> sources = new ArrayList<>();

    private static SwerveDriveSimulation swerveSim = null;

    // Pose and time just before our replay buffer begins
    private static Pose2d poseBeforeHistory = new Pose2d();
    private static double historyStartTime;

    // Recent applied twists, for replay
    private static final Deque<HistoryEntry> history = new ArrayDeque<>();

    private static Pose2d currentPose = new Pose2d();
    private static double lastUpdateTime;

    public static void setSwerveSim(SwerveDriveSimulation sim) {
        swerveSim = sim;
    }

    public static void resetPose(Pose2d newPose) {
        double now = Timer.getFPGATimestamp();
        history.clear();
        poseBeforeHistory = newPose;
        historyStartTime = now;
        currentPose = newPose;
        lastUpdateTime = now;

         TelemetryLogger.logPoseEstimator("Pose reset - X:" + newPose.getX() + " Y:" + newPose.getY()
                + " R:" + newPose.getRotation().getDegrees());

        if (!Robot.isReal() && swerveSim != null) {
            swerveSim.setSimulationWorldPose(newPose);
        }
    }

    /** Add odometry / limelight / any other source */
    public static void addSource(PoseEstimatorSource src) {
        TelemetryLogger.logPoseEstimator("Added source: " + src.name);
        sources.add(src);
    }

    /** Call once per robot loop. Returns the updated pose. */
    public static void update() { 
        double now = Timer.getFPGATimestamp();

        // If any source has a measurement stamped before our last update, we need to
        // replay
        //TODO if you dont delete the data in the sources this if will always be true
        boolean late = sources.stream().anyMatch(s -> s.hasBefore(lastUpdateTime)); 
        if (late) {
            replayHistory();
        }

        // Fuse at 'now', integrate once, push into history, trim old
        Twist2d fused = computeFusedTwist(now);
        if (FiltersConfig.fieldRactangle.contains(currentPose.exp(fused).getTranslation())) {
            currentPose = currentPose.exp(fused);
            history.addLast(new HistoryEntry(now, fused));
            lastUpdateTime = now;
            trimHistory();

            MALog.log("Pose Estimator/Current Pose", currentPose);
        } else {

            TelemetryLogger.logPoseEstimator("Update Rejected Beacus Pose: Outside the field");
        }

    }

    public static Pose2d getCurrentPose() {
        if (currentPose == null) {
            return new Pose2d(-1, -1, new Rotation2d());
        }
        return currentPose;
    }

    /** Backward-compatible overall FOM (mean of XY and θ across sources). */
    public static double getRobotFOM() {
        return sources.stream()
                .mapToDouble(s -> s.getFomAt(lastUpdateTime))
                .average()
                .orElse(0.0);
    }

    /** New: average XY FOM across sources at the last update. */
    public static double getRobotFOMXY() {
        return sources.stream()
                .mapToDouble(s -> s.getFomXYAt(lastUpdateTime))
                .average()
                .orElse(0.0);
    }

    public static Pose2d getPoseLookAhead(double time, ChassisSpeeds speedsRobotRelativ) {
        return currentPose.exp(
                new Twist2d(speedsRobotRelativ.vxMetersPerSecond * time, speedsRobotRelativ.vyMetersPerSecond * time,
                        speedsRobotRelativ.omegaRadiansPerSecond * time));
    }

    // —— internal —— //

    /** Pose reconstructed by applying history up to a query time. */
    public static Pose2d getPoseAt(double queryTime) {
        Pose2d pose = poseBeforeHistory;
        for (HistoryEntry e : history) {
            if (e.time > queryTime)
                break; 
            pose = pose.exp(e.twist);
        }
        return pose;
    }

    /**
     * Replay from poseBeforeHistory through all history entries, slotting in any
     * late packets.
     */
    private static void replayHistory() {
        //TODO i stiil think that the history dont have to be sorted by time and ist better to use
        // https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_time_interpolatable_buffer.html#details
        List<Double> times = history.stream().map(e -> e.time).collect(Collectors.toList());

        Pose2d pose = poseBeforeHistory;
        Deque<HistoryEntry> newHist = new ArrayDeque<>();

        for (double t : times) {
            Twist2d f = computeFusedTwist(t);
            pose = pose.exp(f);
            newHist.addLast(new HistoryEntry(t, f)); 
        }
            //TODO same commants as in the update func and it will be batter to splite the update func to go over history and the "public" update func that call goover
        if (FiltersConfig.fieldRactangle.contains(pose.getTranslation())) {
            history.clear();
            history.addAll(newHist);
            currentPose = pose;
            lastUpdateTime = times.isEmpty() ? historyStartTime : times.get(times.size() - 1);
            trimHistory(); //TODO somthing to think about if the diff after the replay is greate then the max dis the robot can do we split the update with a linear filter of the data
        } else {
            TelemetryLogger.logPoseEstimator("Update Rejected In Replay Beacus Pose: Outside the field or more than one meter in one loop");
        }

    }

    /**
     * Weighted average of each source’s twist at exactly timestamp T (XY vs θ
     * separated).
     */
    private static Twist2d computeFusedTwist(double timestamp) {
        double sumFomXY = 0.0, sumFomTheta = 0.0;
        double dx = 0.0, dy = 0.0, dTheta = 0.0;

        for (PoseEstimatorSource src : sources) { 
            Twist2d tt = src.getTwistAt(timestamp);
            double fXY = src.getFomXYAt(timestamp);
            double fTh = src.getFomThetaAt(timestamp);

            dx += tt.dx * fXY;
            dy += tt.dy * fXY;
            dTheta += tt.dtheta * fTh;

            sumFomXY += fXY;
            sumFomTheta += fTh;
        }
        //TODO need to check the sumFomXY and theta is in a normal range and not close to 0 or 0

        double outDx = dx / sumFomXY;
        double outDy = dy / sumFomXY;
        double outDTh = dTheta / sumFomTheta;

        if (sumFomXY <= 0.0 && sumFomTheta <= 0.0) {
            return new Twist2d();
        }

        return new Twist2d(outDx, outDy, outDTh);
    }

    /** Drop anything older than our window and roll poseBeforeHistory forward. */
    private static void trimHistory() {
        double cutoff = lastUpdateTime - HISTORY_WINDOW_SEC;
        while (!history.isEmpty() && history.peekFirst().time < cutoff) {
            var e = history.removeFirst(); // TODO why use var? 
            poseBeforeHistory = poseBeforeHistory.exp(e.twist);
            historyStartTime = e.time;
        }
        //TODO it will be batter to use an Deque with X cells instade it will also remove the need to trim and you just replac the values head on
    }

    private static class HistoryEntry {
        final double time;
        final Twist2d twist;

        HistoryEntry(double t, Twist2d tw) {
            time = t;
            twist = tw;
        }
    }
}
