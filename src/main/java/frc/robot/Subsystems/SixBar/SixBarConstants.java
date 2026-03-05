
package frc.robot.Subsystems.SixBar;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class SixBarConstants {
        public static final double COLLISION_VOLTS_INSIDE = 0.7;
        public static final double COLLISION_VOLTS_OUTSIDE = 0;
        public static final double DEPLOY_ANGLE = 60;
        public static final double IDLE_ANGLE = 8;
        public static final double BUMPER_ZONE_ANGLE = IDLE_ANGLE;
        public static final double FRAME_PARIMETER_ANGLE = IDLE_ANGLE;
        public static final double COLLISION_TOLERANCE = 10;
        public static final double SHOOTING_ANGLE = 8;
        public static final double COLLISION_POWER_ANGLE = -58;
        public static final double CANT_MOVE_VOLTAGE = 0.0;

        public static final double MINUMUM_ANGLE = -68;
        public static final double MAXIMUM_ANGLE = 5;
        public static final double GEAR = 24;
        public static final double CAN_CODER_GEAR = 2;
        public static final double TOLERANCE = 3;
        public static final double CURRENT_LIMIT = 55;
        public static final double STATOR_CURRENT_LIMIT = 50;
        public static final double CRUSIE_VELO = 1.3;
        public static final double ACCELERATION = 50;

        public static final double TOLERANCE_IN_ARM_BRAKE = 15;
        public static final double COLLISION_DETECTION = 1.5;
        public static final double CLOSE_LOOP_TOLERANCE = -0.5;

        private static final Motor SIXBAR_MOTOR = new Motor(PortMap.SixBarPorts.SIXBAR_MOTOR, MotorType.KRAKEN,
                        "SixBar Motor", InvertedValue.Clockwise_Positive);

        private static final GainConfig REAL_GAIN_CONFIG = new GainConfig().withKP(45).withKI(0).withKD(0);// kp = 36

        public static PositionSystemConstants SIXBAR_CONSTANTS = PositionSystemConstants
                        .newBuilder("SixBar", REAL_GAIN_CONFIG, SIXBAR_MOTOR)
                        .gear(GEAR)
                        .isBrake(false)
                        .tolerance(TOLERANCE)
                        .range(MINUMUM_ANGLE, MAXIMUM_ANGLE)
                        .motionMagic(CRUSIE_VELO, ACCELERATION, 0)
                        .startPose(IDLE_ANGLE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .build();

        public static PositionSystemConstants HOMING_SIXBAR_CONSTANTS = PositionSystemConstants
                        .newBuilder("SixBar", REAL_GAIN_CONFIG, SIXBAR_MOTOR)
                        .gear(GEAR)
                        .isBrake(false)
                        .tolerance(TOLERANCE)
                        .range(MINUMUM_ANGLE, 20)
                        .motionMagic(CRUSIE_VELO, ACCELERATION, 0)
                        .startPose(IDLE_ANGLE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .build();

        public static final State IDLE = new State("IDLE");
        public static final State DEPLOY = new State("DEPLOY");
        public static final State ARMBRAKS = new State("ARMBRAKS");
        public static final State COLLISION = new State("COLLISION", () -> {

        }, () -> {
        });
        public static final State SHOOTING = new State("SHOOTING");

}
