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
    private static final double encoderResolution = 4096;

    private static final double turnGearRatio = 150/7;
    private static final double driveGearRatio = 6.75;


    private static final double driveRot2Meter = Math.PI * 2 * wheelRadius / (encoderResolution * driveGearRatio);
    private static final double driveRPM2MPS = driveRot2Meter / 60;

    private final PIDController driveController = new PIDController(0.000001, 0, 0); // TODO: dummy values plz change later

    private PIDController turnController = new PIDController(0.8, 0.0, 0.0);

    public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, String name, boolean isReversed) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANCoder(canCoderID);
        this.name = name;

        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();       

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(false);
        turnMotor.setInverted(false);
        
        if (isReversed == true) {
            driveMotor.setInverted(true);
        }

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

        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * driveRPM2MPS, new Rotation2d(
                Math.toRadians(canCoder.getAbsolutePosition())
            ));
    }

    public SwerveModulePosition getPosition() {

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
          
        double turnEncoderPosition = (Math.toRadians(canCoder.getAbsolutePosition()) - Math.PI) * -1;
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoderPosition));

        double turnOutput = turnController.calculate(turnEncoderPosition, state.angle.getRadians());


        driveMotorOutput = state.speedMetersPerSecond / DrivetrainSubsystem.maxSpeed;

        SmartDashboard.putNumber(name + " Turn Output", turnOutput);
        SmartDashboard.putNumber(name + " Measurement Desired", state.angle.getRadians());
        SmartDashboard.putNumber(name + " Measurement Actual", turnEncoderPosition);
        SmartDashboard.putNumber(name + " Setpoint", turnController.getSetpoint());
        SmartDashboard.putNumber(name + " Turn Error", turnController.getPositionError());


        driveMotor.set(ControlMode.PercentOutput, driveMotorOutput);
        turnMotor.set(ControlMode.PercentOutput, turnOutput);
    }

}
