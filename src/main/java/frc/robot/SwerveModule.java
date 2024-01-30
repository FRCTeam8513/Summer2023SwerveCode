// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final TalonSRX m_driveMotor;
  private final TalonSRX m_turningMotor;
  private SwerveModuleState lastState = new SwerveModuleState();

  public String name;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.4,0,0);

  // Gains are for example PIDController only - must be determined for your own robot!
  private final PIDController m_turningPIDController =
      new PIDController(
          1,
          0,
          0);


  private final double encoderTicksPerRevolutionSteer = 1656.64;
  private final double encoderTicketPerRevDrive = (6.666*256*4*2*4);
  private final double driveMetersPerRotation = 0.3192;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {
    m_driveMotor = new TalonSRX(driveMotorChannel);
    m_turningMotor = new TalonSRX(turningMotorChannel);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getModuleSpeedOfDriveWheel(), new Rotation2d(getModuleTurningPositionInRad()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getModuleDistanceOfDriveWheel(), new Rotation2d(getModuleTurningPositionInRad()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getModuleTurningPositionInRad()));
    lastState = state;
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getModuleSpeedOfDriveWheel(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getModuleTurningPositionInRad(), state.angle.getRadians());

    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }
  //return the position in degrees
  public double getModuleTurningPositionInRad(){
    double rotations = m_turningMotor.getSelectedSensorPosition() / encoderTicksPerRevolutionSteer;
    double degrees = rotations * 2 * Math.PI;
    return degrees;
  }

  public double getModuleDistanceOfDriveWheel(){
    double meters = (m_driveMotor.getSelectedSensorPosition() / encoderTicketPerRevDrive) * driveMetersPerRotation;
    return meters;
  }

  public double getModuleSpeedOfDriveWheel(){
    double meterspersec = (10 * m_driveMotor.getSelectedSensorVelocity() / encoderTicketPerRevDrive) * driveMetersPerRotation;
    return meterspersec;
  }

  public void updateDashboard(){
    SmartDashboard.putNumber(name + " Module Position", getModuleTurningPositionInRad());
    SmartDashboard.putNumber(name + " Module Velocity", getModuleSpeedOfDriveWheel());
    SmartDashboard.putNumber(name + " Desired Position", lastState.angle.getRadians());
    SmartDashboard.putNumber(name + " Desired Velocity", lastState.speedMetersPerSecond);
  }
}
