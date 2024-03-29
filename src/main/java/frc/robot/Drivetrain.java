// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 4; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI * 3 / 2; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.304, 0.304);
  private final Translation2d m_frontRightLocation = new Translation2d(0.304, -0.304);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.304, 0.304);
  private final Translation2d m_backRightLocation = new Translation2d(-0.304, -0.304);

  private final SwerveModule m_frontLeft = new SwerveModule(4, 7);
  private final SwerveModule m_frontRight = new SwerveModule(1, 5);
  private final SwerveModule m_backLeft = new SwerveModule(2, 6 );
  private final SwerveModule m_backRight = new SwerveModule(3, 8);

  public AHRS navX = new AHRS();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          navX.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_backLeft.name = "Back Left";
    m_backRight.name = "Back Right";
    m_frontLeft.name = "Front Left";
    m_frontRight.name = "Front Right";

    navX.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,navX.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        navX.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public void updateDashboard(){
    m_frontLeft.updateDashboard();
    m_frontRight.updateDashboard();
    m_backLeft.updateDashboard();
    m_backRight.updateDashboard();

    SmartDashboard.putNumber("Gyro", navX.getAngle());
  }

  public void resetGyro(){
    navX.reset();
  }
}
