// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.Other.RobotVersion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** */
public class DriveTrain extends SubsystemBase {

  public SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          Constants.Drive.SMFrontLeftLocation,
          Constants.Drive.SMFrontRightLocation,
          Constants.Drive.SMBackLeftLocation,
          Constants.Drive.SMBackRightLocation);

  public boolean m_WheelLock = false;
  public boolean m_FieldRelativeEnable = true;
  public static final double kMaxSpeed =
      Units.feetToMeters(12.5); // WP this seemed to work don't know why // 3.68
  // meters per second or 12.1
  // ft/s (max speed of SDS Mk3 with Neo motor) // TODO KMaxSpeed needs to go with
  // enum
  public static final double kMaxAngularSpeed =
      Units.rotationsPerMinuteToRadiansPerSecond(
          Constants.Conversion.NeoMaxSpeedRPM / Constants.Conversion.TurnGearRatio); // 1/2
  // rotation
  // per
  // second
  public static final double kMaxTurnAngularSpeed =
      kMaxSpeed / Constants.Drive.SMBackLeftLocation.getNorm(); // 1/2
  // rotation
  // per
  // second
  public static final double kModuleMaxAngularAcceleration =
      Math.PI / 3; // what is this used for again?

  // creates a gyro object. Gyro gives the robots rotation/ where the robot is
  // pointed.
  private final AHRS navx = new AHRS();

  // Creates each swerve module. Swerve modules have a turning and drive motor + a
  // turning and drive encoder.
  public final SwerveModule m_frontRight;
  public final SwerveModule m_frontLeft;
  public final SwerveModule m_backLeft;
  public final SwerveModule m_backRight;

  // Creates an odometry object. Odometry tells the robot its position on the
  // field.
  private final SwerveDriveOdometry m_odometry;

  // Constructor
  /**
   * Our driveTrain Constructor.
   *
   * <p> In here, we: 
   * <ul> <li> Initialize our swerve modules (EX: {@link #m_frontLeft})
   * <li> Set up our autonomous builder (see below)
   * <li> Initialize our odometry: {@link #m_odometry}.
   * </ul>
   * <p> Various other DriveTrain Related things are initalized here too.
   *
   * @param RobotVersion
   */
  public DriveTrain(RobotVersion version) {
    AutoBuilder.configureHolonomic(
        this::getPose2d,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveChassisSpeeds,
        Constants.Drive.pathFollowerConfig,
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

    m_frontRight =
        new SwerveModule(
            Constants.Port.frDriveMtrC,
            Constants.Port.frSteerMtrC,
            Constants.Port.frTurnEncoderDIOC,
            frTurnOffset);
    m_frontLeft =
        new SwerveModule(
            Constants.Port.flDriveMtrC,
            Constants.Port.flSteerMtrC,
            Constants.Port.flTurnEncoderDIOC,
            flTurnOffset);
    m_backLeft =
        new SwerveModule(
            Constants.Port.blDriveMtrC,
            Constants.Port.blSteerMtrC,
            Constants.Port.blTurnEncoderDIOC,
            blTurnOffset);
    m_backRight =
        new SwerveModule(
            Constants.Port.brDriveMtrC,
            Constants.Port.brSteerMtrC,
            Constants.Port.brTurnEncoderDIOC,
            brTurnOffset); // 0.05178

    m_frontLeft.setName("Swerve Module/Front Left");
    m_frontRight.setName("Swerve Module/Front Right");
    m_backLeft.setName("Swerve Module/Back Left");
    m_backRight.setName("Swerve Module/Back Right");
    // initializes odometry
    m_odometry =
        new SwerveDriveOdometry(
            this.m_kinematics, navx.getRotation2d(), getSwerveModulePositions());

    addChild(m_frontLeft.getName(), m_frontLeft);
    addChild(m_frontRight.getName(), m_frontRight);
    addChild(m_backLeft.getName(), m_backLeft);
    addChild(m_backRight.getName(), m_backRight);

    addChild("navx", navx);
  }

  /**
   * Gets our current position in meters on the field.
   *
   * @return A current position on the field.
   *     <p><pi> A translation2d (X and Y on the field) -> {@link #m_kinematics} + A rotation2d (Rot
   *     X and Y on the field) -> {@link #nav}
   */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Gets the Position of the four SwerveModules.
   *
   * <p>This gets the encoder in the motor (drive) and the encoder on the swerve module.
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontLeftPosition = m_frontLeft.getModulePosition();
    SwerveModulePosition frontRightPosition = m_frontRight.getModulePosition();
    SwerveModulePosition backLeftPosition = m_backLeft.getModulePosition();
    SwerveModulePosition backRightPosition = m_backRight.getModulePosition();
    return new SwerveModulePosition[] {
      frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition
    };
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot) {

    SmartDashboard.putNumber(getName() + "/Command/X Speed", xSpeed);
    SmartDashboard.putNumber(getName() + "/Command/Y Speed", ySpeed);
    SmartDashboard.putNumber(getName() + "/Command/Rot Speed", rot);
    SmartDashboard.putBoolean(getName() + "/Command/RobotRelative", m_FieldRelativeEnable);

    Rotation2d robotRotation = new Rotation2d(navx.getRotation2d().getRadians());

    // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            m_FieldRelativeEnable
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
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
    updateOdometry();

    super.periodic();
  }

  /**
   * Runnable Command.
   *
   * <p>Tells the Wheels when to stop or not based off of a boolean varible named {@link
   * #m_WheelLock}.
   *
   * <p>Used in drive Method
   */
  public Command WheelLockCommand() {

    return this.runOnce(
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
   * Runnable Command.
   *
   * <p>Tells the Gyro to reset its heading/which way its facing.
   *
   * <p>Used in drive Method.
   */
  public Command ZeroGyro() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return this.runOnce(
        () -> {
          navx.reset();
        });
  }

  /**
   * Tells the robot to drive based of off a given velocity.
   *
   * <p> Used for Autonomous.
   *
   * @param chassisSpeed (ChassisSpeeds) - this is the desired velocity we would like to drive the
   *     robot.
   */
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeed) {
    drive(
        chassisSpeed.vxMetersPerSecond,
        chassisSpeed.vyMetersPerSecond,
        chassisSpeed.omegaRadiansPerSecond);
  }

  /**
   * Resets the Position of the Odometer, given our Current position.
   *
   * @param pose2d (pose2d) - The current position of the robot on the field. This is a {@link
   *     #resetOdometry(Pose2d)}
   */
  public void resetPose(Pose2d pose2d) {
    resetOdometry(pose2d);
  }

  /**
   * Reset's the Robots Odometry using the Gyro's Current Rotational Position
   *
   * @param pose2d
   */
  public void resetOdometry(Pose2d pose2d) {
    m_odometry.resetPosition(navx.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  /**
   * Converts raw module states into chassis speeds
   *
   * @return chassisSpeeds --> A reading of the speed in m/s our robot is going.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * This command gets the 4 individual SwerveModule States, and groups it into 1 array. <pi> Used
   * for getting our chassis (robots) speed.
   *
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
   * This is a runnable command.
   * <li>This resets the gyro's position.
   * <li>This is needed for Auto, Limelight, and the DriveTrain.
   *
   * @author Jared Forchheimer, Dimitri Lezcano
   * @return Pose2d
   */
  public Command resetPose2d() {
    return this.runOnce(
        () -> {
          resetPose(new Pose2d());
        });
  }

  /**
   * This is a runnable command.
   * <li>This toggles field relative on and off.
   * <li>If
   *
   * @author Jared Forchheimer, Dimitri Lezcano
   * @return Pose2d
   */
  public Command toggleFieldRelativeEnable() {

    return this.runOnce(
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

  public void SetFieldRelativeEnable(boolean fieldRelative) {
    m_FieldRelativeEnable = fieldRelative;
  }

  /** Updates our current Odometry */
  public void updateOdometry() {
    m_odometry.update(navx.getRotation2d(), getSwerveModulePositions());
  }

  /** Stops all the motors on the SwerveModules */
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /** Runnable Command. Runs the {@link #stopModules()} Command. */
  public Command Break() {
    return this.run(
        () -> {
          stopModules();
        });
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Odometry/Pose/X", () -> getPose2d().getX(), null);
    builder.addDoubleProperty("Odometry/Pose/Y", () -> getPose2d().getY(), null);
    builder.addDoubleProperty(
        "Odometry/Pose/Rot", () -> getPose2d().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
        "Odometry/ChassisSpeeds/X", () -> getChassisSpeeds().vxMetersPerSecond, null);
    builder.addDoubleProperty(
        "Odometry/ChassisSpeeds/Y", () -> getChassisSpeeds().vyMetersPerSecond, null);
    builder.addDoubleProperty(
        "Odometry/ChassisSpeeds/Rot",
        () -> Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond),
        null);
    builder.addDoubleProperty(
        "Odometry/navx/Orientation", () -> navx.getRotation2d().getDegrees(), null);
    builder.addBooleanProperty(
        "FieldRelativeEnabled",
        () -> this.m_FieldRelativeEnable,
        (boolean fre) -> m_FieldRelativeEnable = fre);
    m_frontLeft.initSendable(builder);
    m_frontRight.initSendable(builder);
    m_backLeft.initSendable(builder);
    m_backRight.initSendable(builder);
  }

  /** <b> DETAILED EXPLANATION </b> */
  public static int Explanation() {
    return 1;
  }
}
