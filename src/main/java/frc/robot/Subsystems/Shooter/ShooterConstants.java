
package frc.robot.Subsystems.Shooter;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PortMap;
import frc.robot.Commands.ShooterCommand;

public class ShooterConstants {

        public static final double EJECT_VELOCITY = 800;
        public static final double IDLE_VELOCITY = 0;
        public static final double WARMUP_VELOCITY = 300;

        public static final double MAX_VELOCITY = 6060;
        public static final double GEAR = 1;
        public static final double TOLERANCE = 50;
        public static final double CURRENT_LIMIT = 40;
        public static final double STATOR_CURRENT_LIMIT = 30;
        public static final double RAMP_RATE = 0.1;

        public static final double AT_POINT_FOR_FEEDING_TOLERANCE = 300;
        public static final double AT_POINT_FOR_SHOOTING_TOLERANCE = 100;


        public static final Translation2d SHOOTER_OFFSET = new Translation2d(0.0, 0.0);

        private static final Motor MASTER_MOTOR = new Motor(PortMap.ShooterPorts.SHOOTER_MASTER, MotorType.KRAKEN,
                        "Shooter Master Motor", InvertedValue.Clockwise_Positive);
        private static final Motor SLAVE_MOTOR = new Motor(PortMap.ShooterPorts.SHOOTER_SLAVE, MotorType.KRAKEN,
                        "Shooter Slave Motor", InvertedValue.Clockwise_Positive);

        private static final GainConfig REAL_GAIN_CONFIG = new GainConfig().withKS(0.195).withKV(0.118).withKP(0.12);

        public static VelocitySystemConstants SHOOTER_CONSTANTS = VelocitySystemConstants
                        .newBuilder("Shooter", REAL_GAIN_CONFIG, MASTER_MOTOR, SLAVE_MOTOR)
                        .gear(GEAR)
                        .isBrake(false)
                        .tolerance(TOLERANCE)
                        .maxVelocity(MAX_VELOCITY)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .rampRate(RAMP_RATE)
                        .build();

        public static final State IDLE = new State("IDLE");
        public static final State SHOOTING = new State("SHOOTING");
        public static final State WARMUP = new State("WARMUP");
        public static final State FEEDING = new State("FEEDING");
        public static final State FEEDING_IN_MOTION = new State("FEEDING_IN_MOTION");
        public static final State EJECT = new State("EJECT");

}
