// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;

public class Drive extends Command {

  private ChassisSpeeds speeds = new ChassisSpeeds();

  public Drive(double x, double y, double t) {
    speeds.vxMetersPerSecond = x;
    speeds.vyMetersPerSecond = y;
    speeds.omegaRadiansPerSecond = t;
    addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Swerve.getInstance().drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
