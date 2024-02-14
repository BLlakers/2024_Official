// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotVersion2023;
import frc.robot.Constants.RobotVersionConstants;
import frc.robot.Other.RobotVersion;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Represents a swerve drive style drivetrain. */

public class DriveTrainPID extends SubsystemBase {

    public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    Constants.SMFrontLeftLocation,
    Constants.SMFrontRightLocation, 
    Constants.SMBackLeftLocation, 
    Constants.SMBackRightLocation
  );

  public boolean m_WheelLock = false;
  public boolean m_FieldRelativeEnable = true;
  public static final double kMaxSpeed = Units.feetToMeters(12.5); // WP this seemed to work don't know why // 3.68 meters per second or 12.1
  // ft/s (max speed of SDS Mk3 with Neo motor) // TODO KMaxSpeed needs to go with enum
  public static final double kMaxAngularSpeed = Units.rotationsPerMinuteToRadiansPerSecond(Constants.NeoMaxSpeedRPM / Constants.TurnGearRatio); // 1/2 rotation per second
  public static final double kMaxTurnAngularSpeed = kMaxSpeed / Constants.SMBackLeftLocation.getNorm(); // 1/2 rotation per second
  public static final double kModuleMaxAngularAcceleration = Math.PI / 3; // what is this used for again?


  // creates a gyro object. Gyro gives the robots rotation/ where the robot is pointed. 
  private final AHRS navx = new AHRS();

  //Creates each swerve module. Swerve modules have a turning and drive motor + a turning and drive encoder. 
  public final SwerveModule m_frontRight;
  public final SwerveModule m_frontLeft;
  public final SwerveModule m_backLeft;
  public final SwerveModule m_backRight;
  public SwerveDriveKinematics m_initialStates; // TODO unused variable


  // Creates an odometry object. Odometry tells the robot its position on the field.
  private final SwerveDriveOdometry m_odometry;


  /** Flips our position on the field depending on the alliance we are on. Used for Auto.
  */
  public boolean flipFieldPose() {
    // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  // Set up custom logging to add the current path to a field 2d widget
  /*
   * PathPlannerLogging.setLogActivePathCallback((poses) ->
   * field.getObject("path").setPoses(poses));
   * () -> {
   * // Boolean supplier that controls when the path will be mirrored for the red
   * alliance
   * // This will flip the path being followed to the red side of the field.
   * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   * 
   * var alliance = DriverStation.getAlliance();
   * if (alliance.isPresent()) {
   * return alliance.get() == DriverStation.Alliance.Red;
   * }
   * return false;
   * },
   * this
   * );
   */

  // Set up custom logging to add the current path to a field 2d widget
  // PathPlannerLogging.setLogActivePathCallback((poses) ->
  // field.getObject("path").setPoses(poses));

  /** Gets our current position in meters on the field. 
  @return A current position on the field.
  
  * <pi> A translation2d (X and Y on the field) -> {@link #m_kinematics} + A rotation2d (Rot X and Y on the field) -> {@link #nav}
  */
  public Pose2d getPose2d() {
  return m_odometry.getPoseMeters();
  }

  // Constructor 
  /** Our driveTrain Constructor. <p>
   * In here, we initialize our swerve modules (example -> {@link #m_frontLeft}), Get input from autonomous and initialize our odometry -> {@link #m_odometry}. <p> Various other DriveTrain Related thing are initalized here too. 
   * @param RobotVersion
   *
   */
  public DriveTrainPID(RobotVersion version) { 
    AutoBuilder.configureHolonomic(
      
        this::getPose2d,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveChassisSpeeds,
        Constants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
            m_initialStates = new SwerveDriveKinematics(Constants.SMFrontLeftLocation,Constants.SMFrontRightLocation,Constants.SMBackLeftLocation,
       Constants.SMBackRightLocation);
        // sets our wanted offsets. Varies between 2023 and 2024.
    double flTurnOffset = 0, frTurnOffset = 0, blTurnOffset = 0, brTurnOffset = 0;
    if (Constants.defaultRobotVersion == RobotVersion.v2023) {
      flTurnOffset = Constants.RobotVersion2023.flTurnEncoderOffset;
      frTurnOffset = Constants.RobotVersion2023.frTurnEncoderOffset;
      blTurnOffset = Constants.RobotVersion2023.blTurnEncoderOffset;
      brTurnOffset = Constants.RobotVersion2023.brTurnEncoderOffset;
    } else if (Constants.defaultRobotVersion == RobotVersion.v2024) {
      flTurnOffset = Constants.RobotVersion2024.flTurnEncoderOffset;
      frTurnOffset = Constants.RobotVersion2024.frTurnEncoderOffset;
      blTurnOffset = Constants.RobotVersion2024.blTurnEncoderOffset;
      brTurnOffset = Constants.RobotVersion2024.brTurnEncoderOffset;
    }

    m_frontRight = new SwerveModule(
        Constants.frDriveMotorChannel,
        Constants.frSteerMotorChannel,
        Constants.frEncoderChannel,
        frTurnOffset);
    m_frontLeft = new SwerveModule(
        Constants.flDriveMotorChannel,
        Constants.flSteerMotorChannel,
        Constants.flEncoderChannel,
        flTurnOffset);
    m_backLeft = new SwerveModule(
        Constants.blDriveMotorChannel,
        Constants.blSteerMotorChannel,
        Constants.blEncoderChannel,
        blTurnOffset);
    m_backRight = new SwerveModule(
        Constants.brDriveMotorChannel,
        Constants.brSteerMotorChannel,
        Constants.brEncoderChannel,
        brTurnOffset); // 0.05178
//initializes odometry
    m_odometry = new SwerveDriveOdometry(
        this.m_kinematics,
         navx.getRotation2d(),
        getSwerveModulePositions());
      

  }
/**
 * Gets the Position of the four SwerveModules. <p>
 * This gets the encoder in the motor (drive) and the encoder on the swerve module. 
 */
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontLeftPosition = m_frontLeft.getModulePosition();
    SwerveModulePosition frontRightPosition = m_frontRight.getModulePosition();
    SwerveModulePosition backLeftPosition = m_backLeft.getModulePosition();
    SwerveModulePosition backRightPosition = m_backRight.getModulePosition();
    return new SwerveModulePosition[] {
        frontLeftPosition,
        frontRightPosition,
        backLeftPosition,
        backRightPosition
    };
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
  public void drive(double xSpeed, double ySpeed, double rot) {

    SmartDashboard.putNumber("Robot/Command/X Speed", xSpeed);
    SmartDashboard.putNumber("Robot/Command/Y Speed", ySpeed);
    SmartDashboard.putBoolean("Robot/Field Oriented?", m_FieldRelativeEnable);

    Rotation2d robotRotation = new Rotation2d(navx.getRotation2d().getRadians());

    // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        m_FieldRelativeEnable ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    if (!m_WheelLock) {
      setModuleStates(swerveModuleStates);
    } else {
      WheelLock();
    }

  }
/** Tells our modules what speed to go to */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.kDriveMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
/** Tells our wheels to go to the Wheel Locking position (0 m/s, forming an X) */
  public void WheelLock() {
    m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
    m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
    m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
    m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
  }

  @Override
  public void periodic() {
    //updates our Odometry
    updateOdometry();
    //updating our current position
    Pose2d currentPose = this.getPose2d();
    ChassisSpeeds currentChassisSpeeds = this.getChassisSpeeds();
    SmartDashboard.putNumber("Robot/Odometry/Pose X", currentPose.getX());
    SmartDashboard.putNumber("Robot/Odometry/Pose Y", currentPose.getY());
    SmartDashboard.putNumber("Robot/Odometry/Pose Rot", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds X", currentChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds Y", currentChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds Rot",
    Units.radiansToDegrees(currentChassisSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putNumber("Robot/Odometry/navx/Rotation", navx.getRotation2d().getDegrees());

    super.periodic();
  }
/**
 * Runnable Command. <p> Tells the Wheels when to stop or not based off of a boolean varible named {@link #m_WheelLock}. <p> Used in drive Method
 * 
 */
  public Command WheelzLock() {

    return runOnce(
        () -> {

          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (m_WheelLock == true) {
            m_WheelLock = false;
          } else if (m_WheelLock == false) {
            m_WheelLock = true;
          }
        });
  }
/**
 * Runnable Command. <p> Tells the Gyro to reset its heading/which way its facing. <p> Used in drive Method.
 * 
 */
  public Command ZeroGyro() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          navx.reset();

        });
  }
/** Tells the robot to drive based of off a given velocity. <p> Used for Autonomous. 
 * @param chassisSpeed (ChassisSpeeds) - this is the desired velocity we would like to drive the robot.
 *
 */
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeed) {
    drive(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, chassisSpeed.omegaRadiansPerSecond);
  }
/**
 * Resets the Position of the Odometer, given our Current position.
 * @param Pose2d (pose2d) - The current position of the robot on the field. This is a {@link #resetOdometry(Pose2d)} 
 */
  public void resetPose(Pose2d pose2d) {
    resetOdometry(pose2d);

  }
/**
 * Reset's the Robots Odometry using the Gyro's Current Rotational Position
 * 
 * 
 * 
 * @param pose2d 
 */
  public void resetOdometry(Pose2d pose2d) {
    m_odometry.resetPosition(navx.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  /**
   * 
   * Converts raw module states into chassis speeds
   * 
   * @return chassis speeds object
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
  }
/**
 * This command gets the 4 individual SwerveModule States, and groups it into 1 array.
 * <pi> Used for getting our chassis (robots) speed.
 * @author Jared Forchheimer, Dimitri Lezcano
 * @return 4 different SwerveModuleStates
 */
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getModuleState(),
        m_frontRight.getModuleState(),
        m_backLeft.getModuleState(),
        m_backRight.getModuleState()
    };
  }

/**
 * 
 * This is a runnable command. 
 * <li> This resets the gyro's position.
 * <li> This is needed for Auto, Limelight, and the DriveTrain.
 *@author Jared Forchheimer, Dimitri Lezcano
 *@return Pose2d
 */

  public Command resetPose2d() {
    return runOnce(
        () -> {
          resetPose(new Pose2d());
        });
  }
/**
 * 
 * This is a runnable command. 
 * <li> This toggles field relative on and off.
 * <li> If 
 *@author Jared Forchheimer, Dimitri Lezcano
 *@return Pose2d
 */
  public Command toggleFieldRelativeEnable() {

    return runOnce(
        () -> {
          // System.out.println("I am Here");
          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (m_FieldRelativeEnable == true) {
            m_FieldRelativeEnable = false;
            // System.out.println("I am Here 2");
          } else if (m_FieldRelativeEnable == false) {
            m_FieldRelativeEnable = true;
            // System.out.println("I am Here 3");
          }
        });
  }
  /**Updates our current Odometry */
  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        getSwerveModulePositions());
  }
/**Stops all the motors on the SwerveModules */
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }
  /**
   * Runnable Command. Runs the {@link #stopModules()} Command.
   */
  public Command Break(){
  return run(()->{
  stopModules();
    });
  }
}
