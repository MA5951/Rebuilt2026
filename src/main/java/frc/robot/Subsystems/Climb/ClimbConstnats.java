
package frc.robot.Subsystems.Climb;

import java.util.zip.GZIPInputStream;

import com.MAutils.Components.Motor;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.MAutils.Components.Motor.MotorType;

import frc.robot.PortMap;

public class ClimbConstnats {

        public static final double OPEN_POSITION = 10;
        public static final double CLOSE_POSITION = 50;
        public static final double IDLE_POSITION = 20;
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 0;
        public static final double START_POSITION = 0;
        public static final double TOLERANCE = 3;
        public static final double CURRENT_LIMIT = 35;
        public static final double STATOR_CURRENT_LIMIT = 40;
        public static final double CRUISE_VELOCITY = 0;
        public static final double ACCELERATION = 0;

        public static final double AUTONOMOUS_MIN_DISTANCE = 10;
        public static final double INTAKE_OPEN_POSITION = 30;
        public static double LOCK_SERVO_LOCKED_ANGLE = 0;
        public static double LOCK_SERVO_UNLOCKED_ANGLE = 0;
        public static double LOCK_VOLTAGE = 0;
        public static double LOCK_POSITION = 0;

        public static final double KP = 0.1;;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double GEAR = 1;

        public static final double TOLERANCE_FOR_OPENCLOSE_TRIGGER = 3;


        private static final Motor CLIMB_MOTOR = new Motor(PortMap.ClimbPorts.CLIMB_MOTOR, MotorType.KRAKEN,
                        "Climb Motor",
                        InvertedValue.Clockwise_Positive);

        private static final GainConfig GAIN_CONFIG = new GainConfig().withKP(KP).withKI(KI).withKD(KD);

        private static final GainConfig CLOSE_GAIN_CONFIG = new GainConfig().withKP(KP).withKI(KI).withKD(KD);

        public static final PositionSystemConstants CLIMB_CONSTANTS = PositionSystemConstants
                        .newBuilder("Climb", GAIN_CONFIG, CLIMB_MOTOR)
                        .gear(GEAR)
                        .isBrake(true)
                        .tolerance(TOLERANCE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .motionMagic(CRUISE_VELOCITY, ACCELERATION, 0)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .range(MIN_POSITION, MAX_POSITION)
                        .startPose(START_POSITION)
                        .build();

        public static final State IDLE = new State("IDLE");
        public static final State PRECLIMB = new State("PRECLIMB");
        public static final State CLIMB = new State("CLIMB");
        public static final State DOWN = new State("DOWN");
        
        

}
