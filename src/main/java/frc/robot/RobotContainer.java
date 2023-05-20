// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  private final Joystick driver = new Joystick(0); 

  private final DrivetrainSubsystem drive = new DrivetrainSubsystem();

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new RepeatCommand(new RunCommand(
      () -> drive.drive(
          xSpeedLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(0), 0.1)) * DrivetrainSubsystem.maxSpeed,
          ySpeedLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(1), 0.1)) * DrivetrainSubsystem.maxSpeed,
          rotSpeedLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(4), 0.1)) * DrivetrainSubsystem.maxAngularSpeed,
          true
      )
      , drive)));

  }



  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
