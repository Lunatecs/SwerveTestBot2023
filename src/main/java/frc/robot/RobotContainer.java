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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoystickConstants;

public class RobotContainer {
  private final Joystick driver = new Joystick(JoystickConstants.DRIVER_USB); 

  private final DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);



  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    
    drive.setDefaultCommand(new RunCommand(
      () -> drive.drive(
          driver.getRawAxis(JoystickConstants.LEFT_X_AXIS),
          driver.getRawAxis(JoystickConstants.LEFT_Y_AXIS),
          driver.getRawAxis(JoystickConstants.RIGHT_X_AXIS),
          true
      )
      , drive));
    
  }

  public DrivetrainSubsystem getDrive() {
    return drive;
  }

  public void driveWithJoystick(boolean fieldRelative) {

    double xSpeed = driver.getRawAxis(0);

    double ySpeed = driver.getRawAxis(1);

    double rotSpeed = driver.getRawAxis(4);

    drive.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);

  }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
