// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.MAutils.PoseEstimation;

import java.util.NoSuchElementException;
import java.util.Optional;

import com.MAutils.Logger.MALog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Swerve.SwerveConstants;

/** Add your docs here. */
public class PoseEstimatorMA {

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
    MALog.log("Pose Estimator/Kalman/Current Pose", getCurrentPose());
  }

  public static void addOdometryObservation(OdometryObservation observation) {
    // Update odometry pose
    Twist2d twist = SwerveConstants.SWERVE_CONSTANTS.kinematics.toTwist2d(lastWheelPositions,
        observation.startWheelPositions());

    lastWheelPositions = observation.startWheelPositions().clone();

    Pose2d lastOdometryPose = odometryPose;
    MALog.log("Pose Estimator/Kalman/Last Odometry", lastOdometryPose);
    odometryPose = odometryPose.exp(twist);
    MALog.log("Pose Estimator/Kalman/Updated Odometry", odometryPose);

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
      MALog.log("Pose Estimator/Kalman/Odometry Status", "Applied odometry pose estimate");
    } else {
      MALog.log("Pose Estimator/Kalman/Odometry Status", "Invalid odometry pose estimate received; ignoring.");
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
      MALog.log("Pose Estimator/Kalman/Vision Status", "Applied vision pose estimate with Kalman gain");
      
    } else {
      MALog.log("Pose Estimator/Kalman/Vision Status", "Invalid vision pose estimate received; ignoring.");
    }
    
    
  }

  public static boolean validatePose(Pose2d pose) {
    return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians()) && !Double.isInfinite(pose.getX()) && !Double.isInfinite(pose.getY()) && !Double.isInfinite(pose.getRotation().getRadians());
  }
}
