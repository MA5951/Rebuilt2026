
package frc.robot.Subsystems.Transfer;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

public class TransferConstants {
        public static final double IDLE_VOLTAGE = 0;
        public static final double INTAKE_VOLTAGE = 2.0;
        public static final double FEEDING_IN_MOTION_VOLTAGE = 3.0;
        public static final double EJECT_VOLTAGE = 2.0;
        public static final double FEEDING_VOLTAGE = 7.0;       
        public static final double SHOOTING_VOLTAGE = 9.0;
        public static final double UNSTUCK_VOLTAGE = 0.0;
        public static final double MANUAL_VOLTAGE = 7.0;


        public static final double GEAR = 3.33333;
        public static final double RAMP_RATE = 0.2;
        public static final int CURRENT_LIMIT = 55;
        public static final int STATOR_CURRENT_LIMIT = 45;

        private static final Motor TRA_MOTOR = new Motor(
                        PortMap.Transfer_Ports.TRANSFER_MOTOR, MotorType.KRAKEN, "Transfer Motor",
                        InvertedValue.CounterClockwise_Positive);

        public static PowerSystemConstants TRANSFER_CONSTANTS = PowerSystemConstants.builder("Transfer", TRA_MOTOR)
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
