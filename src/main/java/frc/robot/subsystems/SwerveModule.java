// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is the code to run a single swerve module <br>
 * <br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {

  private static final double kPositionConversionFactor =
      (Constants.Conversion.kWheelDiameterM * Math.PI) / Constants.Conversion.DriveGearRatio;
  private static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  // kWheelCircumference used to be
  public static final double kDriveMaxSpeed = Units.feetToMeters(12.5);
  public static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
  public static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final SparkPIDController m_drivePID;

  public final RelativeEncoder m_driveEncoder;
  public final DutyCycleEncoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor
   * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
   * @param turnOffset offset from 0 to 1 for the home position of the encoder
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turnEncoderPWMChannel,
      double turnOffset) {
    // can spark max motor controller objects
    m_driveMotor =
        new CANSparkMax(driveMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();

    m_turningMotor =
        new CANSparkMax(turningMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.setOpenLoopRampRate(0.1);

    m_drivePID = m_driveMotor.getPIDController();
    m_drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_drivePID.setSmartMotionMaxAccel(0.2, 0);

    // spark max built-in encoder
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kPositionConversionFactor); // meters
    m_driveEncoder.setVelocityConversionFactor(kVelocityConversionFactor); // meters per second
    m_driveEncoder.setPosition(0);

    // PWM encoder from CTRE mag encoders
    m_turningEncoder = new DutyCycleEncoder(turnEncoderPWMChannel);
    m_turningEncoder.reset();
    m_turningEncoder.setPositionOffset(turnOffset);
    m_turningEncoder.setDistancePerRotation(2 * Math.PI); // radians ?

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module. <pi> This takes a current velocity for each diffrent
   * drive encoder and a current angle.
   *
   * @return The current state of each Swerve Module. --> The speed and angle of a Module
   */
  public SwerveModuleState getModuleState() {
    // the getVelocity() function normally returns RPM but is scaled in the
    // SwerveModule constructor to return actual wheel speed
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), Rotation2d.fromRadians(m_turningEncoder.getDistance()));
  }

  /**
   * This gets a current Position (Distance per rotation in meters) for each diffrent drive encoder
   * and a current angle from the Duty Cycle encoder.
   *
   * @return The current Position of each Swerve Module
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), Rotation2d.fromRadians(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * <p>This means the speed it should be going and the angle it should be going.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(desiredState, getModulePosition().angle);

    final double signedAngleDifference =
        closestAngleCalculator(
            getModulePosition().angle.getRadians(), optimizedState.angle.getRadians());
    double rotateMotorPercentPower =
        signedAngleDifference / (2 * Math.PI); // proportion error control

    double driveMotorPercentPower = optimizedState.speedMetersPerSecond / kDriveMaxSpeed;
    double turnMotorPercentPower = 1.6 * rotateMotorPercentPower;

    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Drive Encoder/ID: "
            + m_driveMotor.getDeviceId()
            + "/DrivePercent",
        driveMotorPercentPower);
    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Turn Encoder/ID: "
            + m_turningMotor.getDeviceId()
            + "/DrivePercent",
        turnMotorPercentPower);

    m_driveMotor.set(driveMotorPercentPower);
    m_turningMotor.set(turnMotorPercentPower);
  }

  /**
   * Calculates the closest angle and direction between two points on a circle.
   *
   * @param currentAngle
   *     <ul>
   *       <li>where you currently are
   *     </ul>
   *
   * @param desiredAngle
   *     <ul>
   *       <li>where you want to end up
   *     </ul>
   *
   * @return
   *     <ul>
   *       <li>signed double of the angle (rad) between the two points
   *     </ul>
   */
  public double closestAngleCalculator(double currentAngle, double desiredAngle) {
    double signedDiff = 0.0;
    double rawDiff =
        currentAngle > desiredAngle
            ? currentAngle - desiredAngle
            : desiredAngle - currentAngle; // find the positive raw distance between the angles
    double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle
    if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
      signedDiff = ((2 * Math.PI) - modDiff); // full circle minus the angle
      if (desiredAngle > currentAngle)
        signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
    } else {
      signedDiff = modDiff;
      if (currentAngle > desiredAngle) signedDiff = signedDiff * -1;
    }
    return signedDiff;
  }

  /** Tells the drive and turning motor to stop */
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.publishConstInteger("TurnMotor/ID", m_turningMotor.getDeviceId());
    builder.publishConstInteger("DriveMotor/ID", m_driveMotor.getDeviceId());

    builder.addDoubleProperty(
        "TurnMotor/Angle", () -> Units.radiansToDegrees(m_turningEncoder.getDistance()), null);
    builder.addDoubleProperty("DriveMotor/Pos", m_driveEncoder::getPosition, null);
    builder.addDoubleProperty("DriveMotor/Vel", m_driveEncoder::getVelocity, null);

    builder.addDoubleProperty(
        "TurnMotor/Encoder/AbsolutePosition", this.m_turningEncoder::getAbsolutePosition, null);

    builder.setSafeState(this::stop);
  }

  public interface SwerveModuleExplanation {}
}
