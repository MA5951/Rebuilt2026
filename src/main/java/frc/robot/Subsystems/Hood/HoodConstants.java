// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Hood;

import com.MAutils.Components.Motor;
import com.MAutils.Components.Motor.MotorType;
import com.MAutils.RobotControl.State;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Utils.GainConfig;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.PortMap;

/** Add your docs here. */
public class HoodConstants {

    public static final double IDLE_POSITION = 0;
    public static final double EJECT_POSITION = 0;

    public static final double GEAR = 0;
    public static final double POSITION_FACTOR = 0;
    public static final double MOTOR_CURRENT_LIMIT = 0;
    public static final double TOLERANCE = 0;

    public static final Motor HOOD_MOTOR = new Motor(PortMap.HoodPorts.HOOD_MOTOR, MotorType.KRAKEN, "Hood Motor",
            InvertedValue.Clockwise_Positive);

    private static final GainConfig REAL_GAIN_CONFIG = new GainConfig().withKP(0).withKI(0).withKD(0);

    static PositionSystemConstants hoodConstants = PositionSystemConstants
            .newBuilder("Hood", REAL_GAIN_CONFIG, HOOD_MOTOR)
            .positionFactor(POSITION_FACTOR)
            .gear(GEAR)
            .motorCurrentLimit(MOTOR_CURRENT_LIMIT)
            .isBrake(true)
            .statorCurrentLimit(true, MOTOR_CURRENT_LIMIT)
            .tolerance(TOLERANCE)
            .build();

    public static final State IDLE = new State("IDLE");
    public static final State SHOOTING = new State("SHOOTING");
    public static final State FEEDING = new State("FEEDING");
    public static final State STATIC_FEEDING = new State("STATIC_FEEDING");
    public static final State EJECT = new State("EJECT");

}
