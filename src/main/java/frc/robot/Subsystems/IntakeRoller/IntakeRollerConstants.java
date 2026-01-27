
package frc.robot.Subsystems.IntakeRoller;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;


public class IntakeRollerConstants {

     public static final double FORWARD_VOLTAGE = 5;
     public static final double BACKWARD_VOLTAGE = -5;
     public static final double IDLE_VOLTAGE = 0;

     public static final double GEAR = 1;
     public static final double CURRENT_LIMIT = 35;
     public static final double STATOR_CURRENT_LIMIT = 40;
     public static final double RAMP_RATE = 0.2;

     public static final Motor INTAKE_ROLLER_MOTOR = new Motor(PortMap.Intake_Roller_Ports.INTAKE_ROLLER_MOTOR, MotorType.KRAKEN,
     "Intake Roller Motor", InvertedValue.Clockwise_Positive);

     public static final PowerSystemConstants INTAKE_ROLLER_CONSTANTS = PowerSystemConstants.builder("Intake Roller",INTAKE_ROLLER_MOTOR)
     .gear(GEAR)
     .isBrake(false)
     .motorCurrentLimit(CURRENT_LIMIT)
     .statorCurrentLimit(true,STATOR_CURRENT_LIMIT)
     .rampRate(RAMP_RATE)
     .build(PowerSystemConstants::new);
     

     public static final State IDLE = new State("IDLE");
     public static final State FORWARD = new State("FORWARD");
     public static final State BACKWARD = new State("BACKWARD");
}
