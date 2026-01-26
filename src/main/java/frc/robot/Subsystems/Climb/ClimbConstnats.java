
package frc.robot.Subsystems.Climb;

import com.MAutils.Components.Motor;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import frc.robot.PortMap;

public class ClimbConstnats {

    public static final Motor CLIMB_MOTOR = new Motor(PortMap.ClimbPorts.CLIMB_MOTOR, MotorType.KrakenX60,
     "climbMotor", InvertedValue.Clockwise_Positive);

     private static final GainConfig GAIN_CONFIG = new GainConfig().withKP(0);

     public static final PositionSystemConstants CLIMB_CONSTANTS = PositionSystemConstants
     .newBuilder("Climb", GAIN_CONFIG ,CLIMB_MOTOR)
     .gear(0)
     .isBrake(false).tolerance(0)
     .motorCurrentLimit(0)
     .statorCurrentLimit(true,0)
     .build();
     

     public static final State IDLE = new State("IDLE");
     public static final State PRECLIMB = new State("PRECLIMB");
     public static final State CLIMB = new State("CLIMB");
     public static final State DOWN = new State("DOWN");

}
