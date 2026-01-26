package com.MAutils.Vision.Filters;

import java.util.function.Supplier;

import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagFilters {

    // Tag-distance weighting (tune if needed)
    // Full trust up to this distance, then fall off until FAR, then ~0.
    private static final double RANGE_SOFT_TRUST_METERS = 1; // close tags → full trust
    private static final double RANGE_HARD_REJECT_METERS = 5.0; // beyond this → ~0 trust

    private FiltersConfig filtersConfig;
    private final VisionCameraIO visionCameraIO;
    private RawFiducial tag;
    private PoseEstimate visionPose;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private final Supplier<Double> imuYawRadSupplier;

    // ===== CTORS =====
    public AprilTagFilters(FiltersConfig filtersConfig,
                           VisionCameraIO visionCameraIO,
                           Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this(filtersConfig, visionCameraIO, chassisSpeedsSupplier, () -> 0.0);
    }

    public AprilTagFilters(FiltersConfig filtersConfig,
                           VisionCameraIO visionCameraIO,
                           Supplier<ChassisSpeeds> chassisSpeedsSupplier,
                           Supplier<Double> imuYawRadSupplier) {
        this.filtersConfig = filtersConfig;
        this.visionCameraIO = visionCameraIO;
        this.chassisSpeeds = chassisSpeedsSupplier != null
                ? chassisSpeedsSupplier : () -> new ChassisSpeeds();
        this.imuYawRadSupplier = imuYawRadSupplier != null
                ? imuYawRadSupplier : () -> 0.0;
    }

    public void updateFiltersConfig(FiltersConfig newConfig) {
        this.filtersConfig = newConfig;
    }


    //TODO the XY filter can also use the diff yaw filter 
    //TODO need to add speed filter, XY and omega. also to the config
    
    // ===================== XY FOM (more aggressive) =====================
    //TODO must go cleaning 
    public double getXyFOM() {
        // if (1==1) {
        //     return 0;
        // }
        tag = visionCameraIO.getTag();
        visionPose = visionCameraIO.getPoseEstimate(filtersConfig.poseEstimateType);

        // Null safety
        if (visionPose == null || visionPose.pose == null || tag == null) {
            return 0.0;
        }

        // Basic validity checks
        if (!visionCameraIO.isTag()
                || visionPose.pose.getX() == 0
                || visionPose.pose.getY() == 0
                || tag.id == 0
                || !filtersConfig.fieldRactangle.contains(visionPose.pose.getTranslation())
                || tag.ambiguity > filtersConfig.maxAmbiguity
                || visionPose.tagCount < filtersConfig.minTagsSeen) {
            return 0.0;
        }

        // Latency-correct capture timestamp
        final double captureTs = Timer.getFPGATimestamp() - (visionPose.latency / 1000.0);

        // Predicted pose at capture time from our estimator
        final Pose2d predictedAtCapture = PoseEstimator.getPoseAt(captureTs);

        // Motion-cone distance gate (field distance between prediction@capture and LL pose)
        final double dx = visionPose.pose.getX() - predictedAtCapture.getX();
        final double dy = visionPose.pose.getY() - predictedAtCapture.getY();
        final double dist = Math.hypot(dx, dy);

        // Use current chassis speed magnitude (robot-relative norm is fine for bounding)
        ChassisSpeeds speeds = chassisSpeeds.get();
        final double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);


        final double dt = Math.max(1e-3, visionPose.latency / 1000.0);
        final double radius = Math.max(
                    filtersConfig.idleBubbleMeters,
                    v * dt + 0.5 * filtersConfig.aMax * dt * dt
                ) + filtersConfig.motionMarginMeters; //TODO not sure you whant the max, min is making more sense her

     //TODO need to imploment the speed filter(campuer to the chasis)

        // Hard gate: if it’s outside the reachable cone, reject
        // if (dist >= radius) {
        //     return 0.0;
        // }

        // // ---- Aggressive falloff based on maxPoseJumpMeters ----

        // final double maxJump = Math.max(1e-6, filtersConfig.maxPoseJumpMeters);
        // double norm = dist / maxJump; // 0..∞

        // if (norm >= 1.0) {
        //     // Beyond your max sane jump → no trust
        //     return 0.0;
        // }

        // Base trust from motion: quadratic then squared again → much harsher
        // norm = 0   → trust ≈ 1
        // norm = 0.5 → (1 - 0.25)^2 = 0.75^2 ≈ 0.56
        // norm = 0.7 → (1 - 0.49)^2 = 0.51^2 ≈ 0.26
        // double trustMotion = 1.0 - norm * norm;
        // trustMotion = trustMotion * trustMotion; // make curve steeper

        // // ---- Range-based trust: further tags → lower trust ----
        double avgDist = visionPose.avgTagDist; // assumed meters from LL
        // double trustRange;

        // if (avgDist <= RANGE_SOFT_TRUST_METERS) {
        //     trustRange = 1.0; // close tags → full range trust
        // } else if (avgDist >= RANGE_HARD_REJECT_METERS) {
        //     trustRange = 0.0; // very far → no trust
        // } else {
        //     // Smooth cosine drop from 1 → 0 between SOFT and HARD
        //     double x = (avgDist - RANGE_SOFT_TRUST_METERS)
        //             / Math.max(1e-6, (RANGE_HARD_REJECT_METERS - RANGE_SOFT_TRUST_METERS));
        //     trustRange = 0.5 * (1.0 + Math.cos(Math.PI * x));
        // }

        // // Combine motion-based trust and range-based trust
        // double trustXY = trustMotion * trustRange;

        // // If below minimal usefulness, zero it
        // if (trustXY < filtersConfig.minAcceptTrust) {
        //     return 0.0;
        // }


        if (avgDist <= 1) { //TODO add 1 to the config but why its be difult return 0.1? 
            return 0.3;
        } else {
            return 0.1;
        }
        //TODO it will be better to imploment the dis from tag filter as a liner func to the fom

        //return Math.pow((avgDist / 5 ) * 0.1, 2) * 35 ;
        // return 0;
        // return clamp01(trustXY);
    }

    // ===================== θ FOM (unchanged logic) =====================

    public double getOFOM() {
        // Make sure we have a valid pose / tag
        if (visionPose == null || visionPose.pose == null || tag == null) { //TODO you never update the visionPose and tag after the first time
            visionPose = visionCameraIO.getPoseEstimate(filtersConfig.poseEstimateType);
            tag  = visionCameraIO.getTag();

            if (visionPose == null || visionPose.pose == null || tag == null) {
                return 0.0;
            } 
        }

        if (!visionCameraIO.isTag()
                || visionPose.pose.getX() == 0
                || !filtersConfig.fieldRactangle.contains(visionPose.pose.getTranslation())
                || tag.ambiguity > filtersConfig.maxAmbiguity
                || visionPose.tagCount < filtersConfig.minTagsSeen) {
            return 0.0;
        }

        // Hard/soft yaw gate
        final double imuYaw = imuYawRadSupplier.get();
        final double yawLL  = visionPose.pose.getRotation().getRadians(); //TODO just make sure its return the same refrenc angle its can be an angle from a diff cordinat system
        final double yawDiffDeg = Math.toDegrees(Math.abs(angleDiff(yawLL, imuYaw)));

        if (yawDiffDeg >= filtersConfig.hardYawGateDeg) {
            return 0.0; // hard reject
        }

        final double a = filtersConfig.softYawFullTrustDeg;
        final double b = filtersConfig.hardYawGateDeg;

        double trustTheta;
        if (yawDiffDeg <= a) {
            trustTheta = 1.0;
        } else {
            double x = (yawDiffDeg - a) / Math.max(1e-6, (b - a)); // 0..1
            // Smooth cosine drop from 1 → 0 between a..b
            trustTheta = 0.5 * (1.0 + Math.cos(Math.PI * x));
        }

        if (trustTheta < filtersConfig.minAcceptTrust) return 0.0;
        return clamp01(trustTheta);
    }
        //TODO in every fom func you need to have all the filters and not some of them, the reltev robot filter and the dis filter need to be impliment 
        //TODO splite the filters to sparet func and then the code will becam 10 time clenar 

    // ===== helpers =====
    private static double angleDiff(double a, double b) {
        double d = a - b;
        while (d > Math.PI)  d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
