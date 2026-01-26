
package frc.robot.Subsystems.IntakeRoller;

import com.MAutils.Components.Motor;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.utility.LinearPath.State;

import frc.robot.PortMap;

public class IntakeRollerConstants {

     public static final Motor INTAKE_ROLLER_MOTOR = new Motor(PortMap.ClimbPorts.INTAKE_ROLLER_MOTOR, MotorType.KrakenX60,
     "intakerollerMotor", InvertedValue.Clockwise_Positive);

     private static final GainConfig GAIN_CONFIG = new GainConfig().withKP(0);

     public static final PositionSystemConstants INTAKE_ROLLER_CONSTANTS = PositionSystemConstants
     .newBuilder("intakeroller", GAIN_CONFIG ,INTAKE_ROLLER_MOTOR)
     .gear(0)
     .isBrake(false).tolerance(0)
     .motorCurrentLimit(0)
     .statorCurrentLimit(true,0)
     .build();
     

     public static final State IDLE = new State("IDLE");
     public static final State FORWARD = new State("Forward");
     public static final State BACKWARD = new State("BACKWARD");
}
