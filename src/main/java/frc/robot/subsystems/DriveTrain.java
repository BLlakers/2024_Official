// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.*;

import frc.robot.Constants.ChannelConstants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveAndDriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */

public class DriveTrain extends SubsystemBase {

  public PIDController xController = new PIDController(1, 0, 0);
  public PIDController yController = new PIDController(1, 0, 0);
  public ProfiledPIDController m_thetaController = new ProfiledPIDController(1, 0,0, SwerveAndDriveConstants.kChassisthetaContraints);
  private SwerveDriveOdometry odometry;
  private final AHRS gyro = new AHRS();
  private final SwerveModule frontRight = new SwerveModule(ChannelConstants.frDriveMotorChannel,ChannelConstants.frSteerMotorChannel, ChannelConstants.frEncoderChannel, 0.730);
  private final SwerveModule frontLeft = new SwerveModule(ChannelConstants.flDriveMotorChannel, ChannelConstants.flSteerMotorChannel,ChannelConstants.flEncoderChannel, 0.3359);
  private final SwerveModule backLeft = new SwerveModule(ChannelConstants.blDriveMotorChannel, ChannelConstants.blSteerMotorChannel,ChannelConstants.blEncoderChannel, 1.1819);
  private final SwerveModule backRight = new SwerveModule(ChannelConstants.brDriveMotorChannel, ChannelConstants.brSteerMotorChannel,ChannelConstants.brEncoderChannel, 0.9262); // 0.05178
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveAndDriveConstants.frontLeftLocation, SwerveAndDriveConstants.frontRightLocation, SwerveAndDriveConstants.backLeftLocation, SwerveAndDriveConstants.backRightLocation);
  public SwerveDriveKinematics initialStates = new SwerveDriveKinematics(SwerveAndDriveConstants.frontLeftLocation, SwerveAndDriveConstants.frontRightLocation, SwerveAndDriveConstants.backLeftLocation, SwerveAndDriveConstants.backRightLocation);


public boolean flipFieldPose(){

      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

  // Set up custom logging to add the current path to a field 2d widget

    // Set up custom logging to add the current path to a field 2d widget
    //PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    /**
     * This function Turns our position from Meters into Inches <br>
     * <br>
     * Returns an object Pose2d
     * @return Our Current position in Inches
     */
    public Pose2d GetPose2d() {
    Pose2d current_pose_meters = odometry.getPoseMeters();
    Translation2d Translation2d = current_pose_meters.getTranslation().times(ConversionConstants.MetersToInches);
    Pose2d current_pose_inches = new Pose2d(Translation2d, current_pose_meters.getRotation());
    return current_pose_inches;
  }

  // Constructor
  /**
   * Creates a Constructor for the Drivetrain Class
   */
  public DriveTrain() {
    odometry = new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d(), GetModulePositions());
    AutoBuilder.configureHolonomic(
    this::GetPose2d, 
    this::resetPose, 
    this::GetChassisSpeeds, 
    this::driveChassisSpeeds, 
    Constants.pathFollowerConfig,
    this::flipFieldPose,
    this
  );
  }


/**
 * 
 * @return SwerveModulePositions from all 4 modules
 */
  public SwerveModulePosition[] createModules(){
    SwerveModulePosition frontLeftPosition = new SwerveModulePosition(frontLeft.getDifferentState().speedMetersPerSecond, frontLeft.getState().angle);
    SwerveModulePosition frontRightPosition =  new SwerveModulePosition(frontRight.getDifferentState().speedMetersPerSecond, frontRight.getState().angle);
    SwerveModulePosition backLeftPosition =  new SwerveModulePosition(backLeft.getDifferentState().speedMetersPerSecond, backLeft.getState().angle);
    SwerveModulePosition backRightPosition =  new SwerveModulePosition(backRight.getDifferentState().speedMetersPerSecond, backRight.getState().angle);
    return new SwerveModulePosition[]{frontLeftPosition,frontRightPosition,backLeftPosition,backRightPosition};
  }
  
  
  /**
   * Gets the position from all 4 modules
   * @return createModules()
   */
  public SwerveModulePosition[] GetModulePositions() {
    return createModules();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation          Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param defenseHoldingMode Whether we are wheel-locked or not.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative, boolean defenseHoldingMode) {
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putBoolean("Field Oriented?", fieldRelative);
    Rotation2d Rotation2d = new Rotation2d(gyro.getRotation2d().getRadians()); 
    SwerveModuleState[] SwerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d): new ChassisSpeeds(xSpeed, ySpeed, rotation));
    if (!defenseHoldingMode) {
     setModuleStates(SwerveModuleStates);
    } else {
      WheelLock();
    }
  }
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, SwerveAndDriveConstants.FieldRelativeEnable, SwerveAndDriveConstants.WheelLock);
  }
  /** 
   * Tells our robot to go to its desired position.
  */
  public void setModuleStates(SwerveModuleState[] SwerveModuleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, SwerveAndDriveConstants.kChassisMaxSpeed);
     frontRight.setDesiredState(SwerveModuleStates[1]);
      frontLeft.setDesiredState(SwerveModuleStates[0]);
      backLeft.setDesiredState(SwerveModuleStates[2]);
      backRight.setDesiredState(SwerveModuleStates[3]);
  }
  /**
   * Tells our wheels to point towards the middle of the robot.
   */
  public void WheelLock(){
      backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
      frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
  }
  
  public void periodic() {
  updateOdometry();
  Pose2d Pose2d = this.GetPose2d();
  ChassisSpeeds ChassisSpeeds = this.GetChassisSpeeds();
  SmartDashboard.putNumber("CurrentPoseX", Pose2d.getX());
  SmartDashboard.putNumber("CurrentPoseY", Pose2d.getY());
  SmartDashboard.putNumber("CurrentPoseRot", Pose2d.getRotation().getDegrees());
  SmartDashboard.putNumber("chassisSpeedsX", ChassisSpeeds.vxMetersPerSecond);
  SmartDashboard.putNumber("chassisSpeedsY", ChassisSpeeds.vyMetersPerSecond);
  SmartDashboard.putNumber("chassisSpeedsROT", ChassisSpeeds.omegaRadiansPerSecond);
  super.periodic();
  }
  /**
   * Toggles between true and false when a button is pressed.
   * Used for our drive function to determine when we want our wheels to lock or not. 
   * 
   */
  public Command WheelzLock() {
    return runOnce(
        () -> {
          if (SwerveAndDriveConstants.WheelLock == true) {
            SwerveAndDriveConstants.WheelLock = false;
          } else if (SwerveAndDriveConstants.WheelLock == false) {
            SwerveAndDriveConstants.WheelLock = true;
          }
        });
  }
/**
 * Resets the heading (or position) of the gyro.
 */
  public Command ZeroHeading() {
    return runOnce(
      () -> {
       gyro.reset();
      });
  }
/**
 * Resets the position of the robot on the Field.
 * @param Pose2d
 */
  public void resetPose(Pose2d Pose2d) {
    odometry.resetPosition(gyro.getRotation2d(), GetModulePositions(), Pose2d);
  }

/**
   * 
   * Converts raw module states into chassis speeds.
   * 
   * @return chassis speeds object.
   */
  public ChassisSpeeds GetChassisSpeeds(){
    return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates(){
    return new SwerveModuleState[]{
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
  };
  }
/**
 * Resets the Position of the robot on the Field.
 */
  public Command resetPose2d() {
    return runOnce(
      () -> {
        resetPose(new Pose2d());
      });
  }

/**
 * Toggles whether we are in Field Relative mode or not.
 */
  public Command toggleFieldRelativeEnable() {

    return runOnce(
        () -> {
          // System.out.println("I am Here");
          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (SwerveAndDriveConstants.FieldRelativeEnable == true) {
            SwerveAndDriveConstants.FieldRelativeEnable = false;
            // System.out.println("I am Here 2");
          } else if (SwerveAndDriveConstants.FieldRelativeEnable == false) {
            SwerveAndDriveConstants.FieldRelativeEnable = true;
            // System.out.println("I am Here 3");
          }
        });
  }

/**
 * Updates our robots Translation2d and Rotation2d.
 */
  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(),
       GetModulePositions());
  }
/**
 * Tells our SwerveModules to stop moving.
 */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();    
    backRight.stop();
  }
  
  





  //TODO FROM arm.java 2023 code

  public Command RaiseArm() {
    return runOnce(
        () -> {
          // one-time action goes here
          MiscConstants.ArmPosition = MiscConstants.ArmPosition + 1;
          if (MiscConstants.ArmPosition == 4) {
            MiscConstants.ArmPosition = 3;
          }
        });
  }

  public Command LowerArm() {
    return runOnce(
        () -> {
          // one-time action goes here
          MiscConstants.ArmPosition = MiscConstants.ArmPosition - 1;
          if (MiscConstants.ArmPosition == 0) {
            MiscConstants.ArmPosition = 1;
          }
        });
  }
}
