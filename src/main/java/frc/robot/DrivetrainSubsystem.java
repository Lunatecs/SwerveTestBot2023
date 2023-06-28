// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PrivateKey;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static DrivetrainSubsystem drive = null;

  public static final double maxSpeed = 3.0; // 3 meters per second
  public static final double maxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeft = new SwerveModule(DrivetrainConstants.LEFT_FRONT_DRIVE, DrivetrainConstants.LEFT_FRONT_TURN, DrivetrainConstants.LEFT_FRONT_ENC,"LF", false);
  private final SwerveModule frontRight = new SwerveModule(DrivetrainConstants.RIGHT_FRONT_DRIVE, DrivetrainConstants.RIGHT_FRONT_TURN, DrivetrainConstants.RIGHT_FRONT_ENC,"RF", false);
  private final SwerveModule backLeft = new SwerveModule(DrivetrainConstants.LEFT_BACK_DRIVE, DrivetrainConstants.LEFT_BACK_TURN, DrivetrainConstants.LEFT_BACK_ENC,"LB", true);
  private final SwerveModule backRight = new SwerveModule(DrivetrainConstants.RIGHT_BACK_DRIVE, DrivetrainConstants.RIGHT_BACK_TURN, DrivetrainConstants.RIGHT_BACK_ENC,"RB", false);

  private final  WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DrivetrainConstants.PIGEON);

  public static final double FL_HOME = 191.128;
  public static final double FR_HOME = 340.666;
  public static final double BL_HOME = 141.758;
  public static final double BR_HOME = 306.102;

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

  private DrivetrainSubsystem() {
    pigeon.reset();
  }

  public static DrivetrainSubsystem getInstance() {
    if (drive == null)
      drive = new DrivetrainSubsystem();

    return drive;
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
    this.frontLeft.periodic();
    this.frontRight.periodic();
    this.backLeft.periodic();
    this.backRight.periodic();

    // This method will be called once per scheduler run
   /* SmartDashboard.putNumber("FL Drive", frontLeft.driveMotorOutput);
    SmartDashboard.putNumber("FL Turn", frontLeft.turnMotorOutput);

    SmartDashboard.putNumber("FR Drive", frontRight.driveMotorOutput);
    SmartDashboard.putNumber("FR Turn", frontRight.turnMotorOutput);

    SmartDashboard.putNumber("BL Drive", backLeft.driveMotorOutput);
    SmartDashboard.putNumber("BL Turn", backLeft.turnMotorOutput);

    SmartDashboard.putNumber("BR Drive", backRight.driveMotorOutput);
    SmartDashboard.putNumber("BR Turn", backRight.turnMotorOutput);
    */

    /* 
    SmartDashboard.putNumber("FL Abs Position", frontLeft.getAbsoluteEncoderRad() * (180 / Math.PI) % 360);
    SmartDashboard.putNumber("BL Abs Position", backLeft.getAbsoluteEncoderRad() * (180 / Math.PI) % 360);
    SmartDashboard.putNumber("FR Abs Position", frontRight.getAbsoluteEncoderRad() * (180 / Math.PI) % 360);
    SmartDashboard.putNumber("BR Abs Position", backRight.getAbsoluteEncoderRad() * (180 / Math.PI) % 360);
    */
    /* 
    SmartDashboard.putNumber("FL Module Angle", frontLeft.getAngleDeg());
    SmartDashboard.putNumber("FR Module Angle", frontRight.getAngleDeg());
    SmartDashboard.putNumber("BL Module Angle", backLeft.getAngleDeg());
    SmartDashboard.putNumber("BR Module Angle", backRight.getAngleDeg());
    */
  }
}
