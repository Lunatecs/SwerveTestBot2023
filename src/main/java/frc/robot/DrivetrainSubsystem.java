// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public static final double maxSpeed = 3.0; // 3 meters per second
  public static final double maxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeft = new SwerveModule(DrivetrainConstants.LEFT_FRONT_DRIVE, DrivetrainConstants.LEFT_FRONT_TURN, DrivetrainConstants.LEFT_FRONT_ENC);
  private final SwerveModule frontRight = new SwerveModule(DrivetrainConstants.RIGHT_FRONT_DRIVE, DrivetrainConstants.RIGHT_FRONT_TURN, DrivetrainConstants.RIGHT_FRONT_ENC);
  private final SwerveModule backLeft = new SwerveModule(DrivetrainConstants.LEFT_BACK_DRIVE, DrivetrainConstants.LEFT_BACK_TURN, DrivetrainConstants.LEFT_BACK_ENC);
  private final SwerveModule backRight = new SwerveModule(DrivetrainConstants.RIGHT_BACK_DRIVE, DrivetrainConstants.RIGHT_BACK_TURN, DrivetrainConstants.RIGHT_BACK_DRIVE);

  private final  WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DrivetrainConstants.PIGEON);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
    new SwerveDriveOdometry(
        kinematics,
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

  public DrivetrainSubsystem() {
    pigeon.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    odometry.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
