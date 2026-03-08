
package frc.robot.Subsystems.Roller;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class RollerConstants {

        public static final double IDLE_VOLTAGE = 0;
        public static final double INTAKE_VOLTAGE = 8.0;
        public static final double FEEDING_IN_MOTION_VOLTAGE = 3.0;
        public static final double EJECT_VOLTAGE = 3.0;
        public static final double FEEDING_VOLTAGE = 6.0;
        public static final double SHOOTING_VOLTAGE = 6.0;
        public static final double UNSTUCK_VOLTAGE = -2.0;

        public static final double GEAR = 1.5;
        public static final double CURRENT_LIMIT = 35;
        public static final double STATOR_CURRENT_LIMIT = 40;
        public static final double RAMP_RATE = 0.2;

        public static final double MAX_VELOCITY_IN_STOPING = 50.0; // in RPM

        private static final Motor ROLLER_MOTOR = new Motor(PortMap.RollerPorts.ROLLER_MOTOR,
                        MotorType.KRAKEN, "Roller Motor", InvertedValue.Clockwise_Positive);

        public static final PowerSystemConstants ROLLER_CONSTANTS = PowerSystemConstants
                        .builder("Roller", ROLLER_MOTOR)
                        .isBrake(true)
                        .gear(GEAR)
                        .rampRate(RAMP_RATE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .build(PowerSystemConstants::new);

        public static final State IDLE = new State("IDLE");
        public static final State INTAKE = new State("INTAKE");
        public static final State FEEDING = new State("FEEDING");
        public static final State FEEDING_IN_MOTION = new State("FEDDING_IN_MOTION");
        public static final State EJECT = new State("EJECT");
        public static final State SHOOTING = new State("SHOOTING");
        public static final State UNSTUCK = new State("UNSTUCK");
}
