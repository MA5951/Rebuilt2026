
package frc.robot.Subsystems.Climb;


import com.MAutils.Components.Motor;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.MAutils.Components.Motor.MotorType;

import frc.robot.PortMap;

public class ClimbConstnats {

        public static final double OPEN_POSITION = 0.17;
        public static final double CLOSE_POSITION = 0;
        public static final double CLIMB_INTAKE_POSE = 0.08;
        public static final double IDLE_POSITION = 0;
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 0.187;
        public static final double START_POSITION = 0;
        public static final double TOLERANCE = 0.02;
        public static final double CURRENT_LIMIT = 40;
        public static final double STATOR_CURRENT_LIMIT = 40;
        public static final double CRUISE_VELOCITY = 0;
        public static final double ACCELERATION = 0;

        public static final double START_CLOSE_VOLTAGE = -4;
        public static final double END_CLOSE_VOLTAGE = -3;


        public static final double AUTONOMOUS_MIN_DISTANCE = 10;
        public static final double INTAKE_OPEN_POSITION = -55;
        public static final double HOMING_CURRENT_TOLRANCE = 10;
        public static final double HOMING_VOLTAGE = -1;

        public static final double KP = 7;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double POSITION_FACTOR = 0.143 / (360);

        public static final double GEAR = 81;

        public static final double TOLERANCE_FOR_OPENCLOSE_TRIGGER = 0.03;


        private static final Motor CLIMB_MOTOR = new Motor(PortMap.ClimbPorts.CLIMB_MOTOR, MotorType.KRAKEN,
                        "Climb Motor",
                        InvertedValue.CounterClockwise_Positive);

        private static final GainConfig GAIN_CONFIG = new GainConfig().withKP(KP).withKI(KI).withKD(KD);

        public static final PositionSystemConstants CLIMB_CONSTANTS = PositionSystemConstants
                        .newBuilder("Climb", GAIN_CONFIG, CLIMB_MOTOR)
                        .gear(GEAR)
                        .isBrake(false)
                        .tolerance(TOLERANCE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .range(MIN_POSITION, MAX_POSITION)
                        .startPose(START_POSITION)
                        .positionFactor(POSITION_FACTOR)
                        .build();

        public static final PositionSystemConstants HOMING_CLIMB_CONSTANTS = PositionSystemConstants
                        .newBuilder("Climb", GAIN_CONFIG, CLIMB_MOTOR)
                        .gear(GEAR)
                        .isBrake(false)
                        .tolerance(TOLERANCE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(false, STATOR_CURRENT_LIMIT)
                        .range(-0.18, MAX_POSITION)
                        .startPose(START_POSITION)
                        .positionFactor(POSITION_FACTOR)
                        .build();

        public static final State IDLE = new State("IDLE");
        
        public static final State DOWN = new State("DOWN");



}
