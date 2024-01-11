// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */

public class DriveTrainPID extends SubsystemBase {
  public boolean WheelLock = false;
  public boolean FieldRelativeEnable = true;
  public static final double kMaxSpeed = 1; // WP this seemed to work don't know why // 3.68 meters per second or 12.1
                                            // ft/s (max speed of SDS Mk3 with Neo motor)
  public static final double kMaxAngularSpeed = Math.PI / 3; // 1/2 rotation per second
  private final AHRS navx = new AHRS();

  private final Translation2d m_frontRightLocation = new Translation2d(0.285, -0.285);
  private final Translation2d m_frontLeftLocation = new Translation2d(0.285, 0.285);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.285, 0.285);
  private final Translation2d m_backRightLocation = new Translation2d(-0.285, -0.285);

  // constructor for each swerve module
  public final SwerveModule m_frontRight = new SwerveModule(Constants.frDriveMotorChannel,
      Constants.frSteerMotorChannel, Constants.frEncoderChannel, 0.730);
  public final SwerveModule m_frontLeft = new SwerveModule(Constants.flDriveMotorChannel, Constants.flSteerMotorChannel,
      Constants.flEncoderChannel, 0.3359);
  public final SwerveModule m_backLeft = new SwerveModule(Constants.blDriveMotorChannel, Constants.blSteerMotorChannel,
      Constants.blEncoderChannel, 1.1819);
  public final SwerveModule m_backRight = new SwerveModule(Constants.brDriveMotorChannel, Constants.brSteerMotorChannel,
      Constants.brEncoderChannel, 0.9262); // 0.05178
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // INITIAL POSITIONS to help define swerve drive odometry. THis was a headache
  public SwerveDriveKinematics m_initialStates;

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      navx.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public Pose2d GetPose2d() {
    Pose2d current_pose_meters = m_odometry.getPoseMeters();
    Pose2d current_pose_inches = (current_pose_meters.times(Constants.MetersToInches));
    return current_pose_inches;
  }

  // Constructor
  public DriveTrainPID() {
    m_initialStates = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean defenseHoldingMode) {
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putBoolean("Field Oriented?", fieldRelative);
    // double angleOffset = DriverStation.getAlliance().toString() == "Blue" ?
    // Math.PI/2 : -Math.PI/2;
    // SmartDashboard.putString("isRed",
    // DriverStation.getAlliance().toString());//NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getString("bruh"));
    // SmartDashboard.putNumber( "angleOffset", angleOffset);
    Rotation2d robotRotation = new Rotation2d(navx.getRotation2d().getRadians()); // + angleOffset);
                                                                                  // //DriverStation.getAlliance() ==
                                                                                  // Alliance.Blue ? new
                                                                                  // Rotation2d(navx.getRotation2d().getDegrees()
                                                                                  // + 180) : navx.getRotation2d();
    // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
    var swerveModuleStates = m_kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // System.out.println(defenseHoldingMode);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    if (!defenseHoldingMode) {
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
      m_backRight.setDesiredState(swerveModuleStates[3]);
    } else {
      m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
      m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
    }

  }
@Override
public void periodic() {
    Pose2d curentPose = this.GetPose2d();
  SmartDashboard.putNumber("CurrentPoseX",curentPose.getX());
  SmartDashboard.putNumber("CurrentPoseY",curentPose.getY());
  SmartDashboard.putNumber("CurrentPoseRot",curentPose.getRotation().getDegrees());
    super.periodic();
}
  public Command WheelzLock() {

    return runOnce(
        () -> {

          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (WheelLock == true) {
            WheelLock = false;
          } else if (WheelLock == false) {
            WheelLock = true;
          }
        });
  }

  public Command ZeroGyro() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          navx.reset();

        });
  }

  public Command toggleFieldRelativeEnable() {

    return runOnce(
        () -> {
          // System.out.println("I am Here");
          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (FieldRelativeEnable == true) {
            FieldRelativeEnable = false;
            // System.out.println("I am Here 2");
          } else if (FieldRelativeEnable == false) {
            FieldRelativeEnable = true;
            // System.out.println("I am Here 3");
          }
        });
  }

  /**
   * 
   * Converts raw module states into chassis speeds
   * 
   * @return chassis speeds object
   */
  /*
   * public ChassisSpeeds getChassisSpeeds() {
   * return m_kinematics.toChassisSpeeds(m_backLeft.getState(),
   * m_frontLeft.getState(), m_backRight.getState(), m_frontRight.getState());
   * }
   */

  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }
}
