
package frc.robot.Subsystems.Shooter;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class ShooterConstants {

    public static final double FEED_FORWARD = 0;
    public static final double EJECT_VELOCITY = 0;
    public static final double IDLE_VELOCITY = 0;
    public static final double WARMUP_VELOCITY = 0;

    public static final Motor MASTER_MOTOR = new Motor(PortMap.ShooterPorts.SHOOTER_MASTER, MotorType.KRAKEN,
            "Shooter Master Motor", InvertedValue.Clockwise_Positive);
    public static final Motor SLAVE_MOTOR = new Motor(PortMap.ShooterPorts.SHOOTER_SLAVE, MotorType.KRAKEN,
            "Shooter Slave Motor", InvertedValue.Clockwise_Positive);

    private static final GainConfig REAL_GAIN_CONFIG = new GainConfig().withKP(0).withKI(0).withKD(0);

    public static VelocitySystemConstants shooterConstants = VelocitySystemConstants
            .newBuilder("Shooter", REAL_GAIN_CONFIG, MASTER_MOTOR, SLAVE_MOTOR)
            .gear(0)
            .isBrake(false)
            .tolerance(0)
            .motorCurrentLimit(0)
            .maxVelocity(0)
            .statorCurrentLimit(true, 0)
            .build();

    public static final State IDLE = new State("IDLE");
    public static final State SHOOTING = new State("SHOOTING");
    public static final State WARMUP = new State("WARMUP");
    public static final State FEEDING = new State("FEEDING");
    public static final State STATIC_FEEDING = new State("STATIC_FEEDING");
    public static final State EJECT = new State("EJECT");

}
