// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.MAutils.RobotControl.SubsystemCommand;

import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;

/** Add your docs here. */
public class HoodCommand extends SubsystemCommand {
    private static final Hood hood = Hood.getInstance();

    public HoodCommand() {
        super(hood);
        addRequirements(hood);
    }

    @Override
    public void Automatic() {
        switch (hood.getCurrentState().stateName) {
            case "IDLE":
                hood.setPosition(HoodConstants.IDLE_POSITION);
                break;
            case "SHOOTING":
                hood.setPosition(Superstructure.getHoodShootingPosition());// TODO after merge
                break;
            case "FEEDING":
                hood.setPosition(Superstructure.getHoodFeedingPosition());// TODO after merge
                break;
            case "STATIC_FEEDING":
                hood.setPosition(Superstructure.getHoodStaticFeedingPosition());// TODO after merge
                break;
            case "EJECT":
                hood.setPosition(HoodConstants.EJECT_POSITION);
                break;
        }
    }

    @Override
    public void Manual() {

    }

    @Override
    public void CantMove() {
        hood.setVoltage(0);
    }
}
