// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Hood;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PositionControlledSystem;

/** Add your docs here. */
public class Hood extends PositionControlledSystem {

    private static Hood hood;

    private Hood() {
        super(HoodConstants.HOOD_CONSTANTS, HoodConstants.IDLE, HoodConstants.EJECT, HoodConstants.FEEDING,
                HoodConstants.SHOOTING, HoodConstants.FEEDING_IN_MOTION);

    }

    @Override
    public void createSelfTest() {

    }

    @Override
    public boolean CAN_MOVE() {
        return true;
    }

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }
}
