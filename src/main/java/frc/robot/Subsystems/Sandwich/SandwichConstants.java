
package frc.robot.Subsystems.Sandwich;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class SandwichConstants {

        public static final double IDLE_VOLTAGE = 0;
        public static final double INTAKE_VOLTAGE = 8.0;
        public static final double FEDDING_IN_MOTION_VOLTAGE = 8.0;
        public static final double EJECT_VOLTAGE = 8.0;
        public static final double FEEDING_VOLTAGE = 8.0;
        public static final double SHOOTING_VOLTAGE = 8.0;
        public static final double UNSTUCK_VOLTAGE = -8.0;

        public static final double GEAR = 12.0 / 60.0;
        public static final double RAMP_RATE = 0.2; 
        public static final int CURRENT_LIMIT = 30;
        public static final int STATOR_CURRENT_LIMIT = 40;

        public static final double IS_BALLS_DISTANCE = 0.1;

        private static final Motor SANDWICH_MOTOR = new Motor(
                        PortMap.Sandwich_Ports.SANDWICH_MOTOR, MotorType.KRAKEN, "Sandwich Motor",
                        InvertedValue.Clockwise_Positive);

        

        public static PowerSystemConstants SANDWICH_CONSTANTS = PowerSystemConstants.builder("Sandwich", SANDWICH_MOTOR)
                        .gear(GEAR)
                        .isBrake(true)
                        .rampRate(RAMP_RATE)
                        .motorCurrentLimit(CURRENT_LIMIT)
                        .statorCurrentLimit(true, STATOR_CURRENT_LIMIT)
                        .build(PowerSystemConstants::new);

        public static final State IDLE = new State("IDLE");
        public static final State INTAKE = new State("INTAKE");
        public static final State FEEDING = new State("FEEDING");
        public static final State FEEDING_IN_MOTION = new State("FEEDING_IN_MOTION");
        public static final State EJECT = new State("EJECT");
        public static final State SHOOTING = new State("SHOOTING");
        public static final State UNSTUCK = new State("UNSTUCK");

}
