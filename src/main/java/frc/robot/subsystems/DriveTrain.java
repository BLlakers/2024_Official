// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.util.PathPlannerLogging;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.pathfinding.*;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveAndDriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */

public class DriveTrain extends SubsystemBase {
  public static SwerveDriveOdometry odometry;

  /*AutoBuilder autoBuilder = AutoBuilder.configureHolonomic(
    this::getPose, 
    this::resetPose, 
    this::getSpeeds, 
    this::driveRobotRelative, 
    Constants.pathFollowerConfig,
    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
    this
  );

  // Set up custom logging to add the current path to a field 2d widget
  /*PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );*/

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
    odometry = new SwerveDriveOdometry(SwerveAndDriveConstants.kinematics, SwerveAndDriveConstants.gyro.getRotation2d(), GetModulePositions());
  }


/**
 * 
 * @return SwerveModulePositions from all 4 modules
 */
  public SwerveModulePosition[] createModules(){
    SwerveModulePosition frontLeftPosition = new SwerveModulePosition(SwerveAndDriveConstants.frontLeft.getDifferentState().speedMetersPerSecond, SwerveAndDriveConstants.frontLeft.getState().angle);
    SwerveModulePosition frontRightPosition =  new SwerveModulePosition(SwerveAndDriveConstants.frontRight.getDifferentState().speedMetersPerSecond, SwerveAndDriveConstants.frontRight.getState().angle);
    SwerveModulePosition backLeftPosition =  new SwerveModulePosition(SwerveAndDriveConstants.backLeft.getDifferentState().speedMetersPerSecond, SwerveAndDriveConstants.backLeft.getState().angle);
    SwerveModulePosition backRightPosition =  new SwerveModulePosition(SwerveAndDriveConstants.backRight.getDifferentState().speedMetersPerSecond, SwerveAndDriveConstants.backRight.getState().angle);
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
    Rotation2d Rotation2d = new Rotation2d(SwerveAndDriveConstants.gyro.getRotation2d().getRadians()); 
    SwerveModuleState[] SwerveModuleStates = SwerveAndDriveConstants.kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d): new ChassisSpeeds(xSpeed, ySpeed, rotation));
    if (!defenseHoldingMode) {
     setModuleStates(SwerveModuleStates);
    } else {
      WheelLock();
    }
  }
  /** 
   * Tells our robot to go to its desired position.
  */
  public void setModuleStates(SwerveModuleState[] SwerveModuleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, SwerveAndDriveConstants.kMaxSpeed);
     SwerveAndDriveConstants.frontRight.setDesiredState(SwerveModuleStates[1]);
      SwerveAndDriveConstants.frontLeft.setDesiredState(SwerveModuleStates[0]);
      SwerveAndDriveConstants.backLeft.setDesiredState(SwerveModuleStates[2]);
      SwerveAndDriveConstants.backRight.setDesiredState(SwerveModuleStates[3]);
  }
  /**
   * Tells our wheels to point towards the middle of the robot.
   */
  public void WheelLock(){
      SwerveAndDriveConstants.backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
      SwerveAndDriveConstants.frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      SwerveAndDriveConstants.backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
      SwerveAndDriveConstants.frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
  }
  
  @Override
  
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
        SwerveAndDriveConstants.gyro.reset();
      });
  }
/**
 * Resets the position of the robot on the Field.
 * @param Pose2d
 */
  public void resetPose(Pose2d Pose2d) {
    odometry.resetPosition(SwerveAndDriveConstants.gyro.getRotation2d(), GetModulePositions(), Pose2d);
  }

/**
   * 
   * Converts raw module states into chassis speeds.
   * 
   * @return chassis speeds object.
   */
  public ChassisSpeeds GetChassisSpeeds(){
    return SwerveAndDriveConstants.kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates(){
    return new SwerveModuleState[] {
      SwerveAndDriveConstants.frontLeft.getState(),
      SwerveAndDriveConstants.frontRight.getState(),
      SwerveAndDriveConstants.backLeft.getState(),
      SwerveAndDriveConstants.backRight.getState()
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
    odometry.update(SwerveAndDriveConstants.gyro.getRotation2d(),
       GetModulePositions());
  }
/**
 * Tells our SwerveModules to stop moving.
 */
  public void stopModules() {
    SwerveAndDriveConstants.frontLeft.stop();
    SwerveAndDriveConstants.frontRight.stop();
    SwerveAndDriveConstants.backLeft.stop();    
    SwerveAndDriveConstants.backRight.stop();
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
