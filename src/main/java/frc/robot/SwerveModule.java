// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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
    
    private static final double wheelRadius = 0.0508;
    private static final double encoderResolution = 4096; // 2048 for TalonFX

    private static final double turnGearRatio = 150/7;
    private static final double driveGearRatio = 6.75; // TODO: change later

    //private static final double TICKS_PER_ROT = 2 * Math.PI * wheelRadius / encoderResolution;
    //private static final double TICKS_PER_RAD = 2 * Math.PI / encoderResolution;

    private static final double driveRot2Meter = Math.PI * 2 * wheelRadius / (encoderResolution * driveGearRatio);
    // private static final double turnRot2Rad = 2 * Math.PI / (encoderResolution * turnGearRatio);
    private static final double turnRot2Rad = 2 * Math.PI / 360;
    // private static final double turnRot2Deg = 360 / (encoderResolution * turnGearRatio);
    private static final double driveRPM2MPS = driveRot2Meter / 60;
    // private static final double turnRPM2RPS = turnRot2Rad / 60;

    private static final double moduleMaxAngularVelocity = DrivetrainSubsystem.maxAngularSpeed;
    private static final double moduleMaxAngularAcceleration = 2 * Math.PI;

    private final PIDController driveController = new PIDController(0.000001, 0, 0); // TODO: dummy values plz change later

    // private final ProfiledPIDController turnController =
    //     new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

    private PIDController turnController = new PIDController(0.8, 0.0, 0.0);

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, String name, boolean isReversed) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANCoder(canCoderID);
        this.name = name;

        // canCoder.configFactoryDefault();
        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();       

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(false);
        turnMotor.setInverted(false);

        if (isReversed == true) {
            driveMotor.setInverted(true);
        }


        // turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //driveMotor.config_kP(0,.5);
        //driveMotor.config_kI(0, 0.0);
        //driveMotor.config_kD(0, 0.0);
        //driveMotor.setSelectedSensorPosition(0);

        //turnMotor.config_kP(0, 0.01);

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset Encoders
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(0);

        SmartDashboard.setDefaultNumber(name + " Turn Output", 0);
        SmartDashboard.setDefaultNumber(name + " Sensor Position", 0);
        
        SmartDashboard.setDefaultNumber(name + " Measurement Desired", 0);
        SmartDashboard.setDefaultNumber(name + " Measurement Actual", 0);
        SmartDashboard.setDefaultNumber(name + " Setpoint", 0);
        SmartDashboard.setDefaultNumber(name + " Turn Error", 0);
    }
    
    public SwerveModuleState getState() {
        // this code is adapted from the wpilib example code to work with falcons
        // will need to check if it actually works

        /* 
        SmartDashboard.putNumber("Drive State (Conversion) " + name, driveMotor.getSelectedSensorVelocity() * driveRPM2MPS);
        SmartDashboard.putNumber("Drive State " + name, driveMotor.getSelectedSensorVelocity());
        */
        //SmartDashboard.putNumber("Turn State (Conversion) " + name, turnMotor.getSelectedSensorPosition() * turnRot2Deg);
        //SmartDashboard.putNumber("Turn State " + name, turnMotor.getSelectedSensorPosition());

        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * driveRPM2MPS, new Rotation2d(
                Math.toRadians(canCoder.getAbsolutePosition())
            ));
    }

    public SwerveModulePosition getPosition() {
        // same story here
        /* 
        SmartDashboard.putNumber("Drive Position (Conversion) " + name, driveMotor.getSelectedSensorPosition() * driveRot2Meter);
        SmartDashboard.putNumber("Drive Position " + name, driveMotor.getSelectedSensorPosition());
        */
        //SmartDashboard.putNumber("Turn Position (Conversion) " + name, turnMotor.getSelectedSensorPosition() * turnRot2Rad);
        //SmartDashboard.putNumber("Turn Position " + name, turnMotor.getSelectedSensorPosition());

        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * driveRot2Meter, new Rotation2d(
            Math.toRadians(canCoder.getAbsolutePosition())
        ));
    }

    public void periodic() {
        SmartDashboard.putNumber(name + " Sensor Position", Math.toRadians(canCoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double driveMotorOutput;
        double turnMotorOutput;

        // if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
        //     driveMotor.set(ControlMode.PercentOutput, 0);
        //     turnMotor.set(ControlMode.PercentOutput, 0);
        //     return;
        // }

        // funi optimizaiton
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnMotor.getSelectedSensorPosition() * turnRot2Rad));
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(
                //Math.toRadians(canCoder.getAbsolutePosition())
           // ));
           SwerveModuleState state = desiredState;

        //final double driveOutput = driveController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
        //final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);
        double turnEncoderPosition = (Math.toRadians(canCoder.getAbsolutePosition()) - Math.PI) * -1;
        double turnOutput = turnController.calculate(turnEncoderPosition, state.angle.getRadians());
        //final double turnFF = turnFeedforward.calculate(turnController.getSetpoint().velocity);

        driveMotorOutput = state.speedMetersPerSecond / DrivetrainSubsystem.maxSpeed;//13.0; //+ driveFF;
        // turnMotorOutput = turnOutput;//13.0; //+ turnFF;

        SmartDashboard.putNumber(name + " Turn Output", turnOutput);
        //SmartDashboard.putNumber(name + " Drive Output", driveMotorOutput);
        
        SmartDashboard.putNumber(name + " Measurement Desired", state.angle.getRadians());
        SmartDashboard.putNumber(name + " Measurement Actual", turnEncoderPosition);
        SmartDashboard.putNumber(name + " Setpoint", turnController.getSetpoint());
        SmartDashboard.putNumber(name + " Turn Error", turnController.getPositionError());

        //SmartDashboard.putNumber(name + " Module Angle", getAngleDeg());

        // honestly, im just going off of wpilib
        //driveMotor.set(ControlMode.Current, 5.0);// driveoutput + driveff
        driveMotor.set(ControlMode.PercentOutput, driveMotorOutput);
        turnMotor.set(ControlMode.PercentOutput, turnOutput);
        //turnMotor.set(ControlMode.Current, turnutput + turnFF);

        //SmartDashboard.putNumber("Drive Current Output " + name, this.driveMotor.getSupplyCurrent());
        //SmartDashboard.putNumber("DriveFF "+ name, turnFF);
    }

}
