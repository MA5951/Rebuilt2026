
package frc.robot.Subsystems.Hood;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class HoodConstants {

        public static final double IDLE_POSITION = 0;
        public static final double EJECT_POSITION = 10;
        public static final double MIN_POSITION = -1.5;
        public static final double MAX_POSITION = 28;
        public static final double GEAR = 185.185;
        public static final double CAN_CODER_GEAR = 18.656;
        public static final double MOTOR_CURRENT_LIMIT = 20;
        public static final double STATOR_CURRENT_LIMIT = 20;
        public static final double TOLERANCE = 1;

        public static double DELTA_CURRENT = 15;
        public static final double HOOMING_VOLTAGE = 0;


        private static final Motor HOOD_MOTOR = new Motor(PortMap.HoodPorts.HOOD_MOTOR, MotorType.KRAKEN, "Hood Motor",
                        InvertedValue.CounterClockwise_Positive);

        private static final GainConfig REAL_GAIN_CONFIG = new GainConfig().withKP(300).withKI(0).withKD(0);

        public static PositionSystemConstants HOOD_CONSTANTS = PositionSystemConstants
                        .newBuilder("Hood", REAL_GAIN_CONFIG, HOOD_MOTOR)
                        .gear(GEAR)
                        .motorCurrentLimit(MOTOR_CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .startPose(IDLE_POSITION)
                        .isBrake(true)
                        .range(MIN_POSITION, MAX_POSITION)
                        .tolerance(TOLERANCE)
                        .build();

        public static final State IDLE = new State("IDLE");
        public static final State SHOOTING = new State("SHOOTING");
        public static final State FEEDING_IN_MOTION = new State("FEEDING_IN_MOTION");
        public static final State FEEDING = new State("FEEDING");
        public static final State EJECT = new State("EJECT");
        public static final double MANUAL_INCREMENT = 0;
        public static final State HOMING = new State("HOMING");

}
