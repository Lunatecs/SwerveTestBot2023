// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveModule {
    
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANCoder canCoder;

    private String name;
    
    private static final double wheelRadius = 2;
    private static final double encoderResolution = 2048.0;

    private static final double TICKS_PER_ROT = 2 * Math.PI * wheelRadius / encoderResolution;
    private static final double TICKS_PER_RAD = 2 * Math.PI / encoderResolution;

    private static final double moduleMaxAngularVelocity = DrivetrainSubsystem.maxAngularSpeed;
    private static final double moduleMaxAngularAcceleration = 2 * Math.PI;

    CANCoderConfiguration config = new CANCoderConfiguration();

    public double driveMotorOutput;
    public double turnMotorOutput;

    //canCoder.configAllSettings(config);
    //Here, we ran into an issue where we were unable to use CANCoderConfiguration()
    
    private final PIDController driveController = new PIDController(1, 0, 0); // dummy values plz change later

    private final ProfiledPIDController turnController = new ProfiledPIDController(
                                                                                1, 
                                                                                0,
                                                                                0,
                                                                                new TrapezoidProfile.Constraints(moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3); // idk what happens here
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5); // just trusting in wpilib

    public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, String name) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANCoder(canCoderID);
        this.name=name;

        /*
         * wpi had code to set up encoders
         * idk what we need to do for falcons
         * that is a thursday problem
         * - sujit
         * 
         * I think its allg, but im not sure
         * - future sujit
         */

        driveMotor.configFactoryDefault();
        driveMotor.config_kP(0,.01);


        turnMotor.configFactoryDefault();

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        canCoder.configFactoryDefault();
        config.sensorCoefficient = 2.0 * Math.PI / 2048.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        canCoder.configAllSettings(config);
    }

    public SwerveModuleState getState() {
        // this code is adapted from the wpilib example code to work with falcons
        // will need to check if it actually works

        // i used cancoder.getAbsolutePosition() and it may not be allg - sujit
        //it might be good???
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * TICKS_PER_ROT, new Rotation2d(canCoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        // same story here
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * TICKS_PER_ROT, new Rotation2d(canCoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // funi optimizaiton
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(canCoder.getAbsolutePosition()));


        final double driveOutput = driveController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
        final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);


        final double turnOutput = turnController.calculate(canCoder.getAbsolutePosition(), state.angle.getRadians());
        final double turnFF = turnFeedforward.calculate(turnController.getSetpoint().velocity);

        driveMotorOutput = driveOutput + driveFF;
        turnMotorOutput = turnOutput + turnFF;

       // SmartDashboard.putNumber("Turn Output " + name, turnMotorOutput);
        SmartDashboard.putNumber("Drive Output " + name, driveMotorOutput);
        

        // honestly, im just going off of wpilib
        driveMotor.set(ControlMode.Current, driveOutput + driveFF);
        turnMotor.set(ControlMode.PercentOutput, 0.0);
        //turnMotor.set(ControlMode.Current, turnOutput + turnFF);

        SmartDashboard.putNumber("Drive Current Output " + name, this.driveMotor.getSupplyCurrent());
    }

}
