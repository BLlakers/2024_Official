// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType; //TODO Was giving us weird error 
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
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

    private static final double kPositionConversionFactor = (Constants.kWheelDiameterM * Math.PI) / Constants.GearRatio;
    private static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    // kWheelCircumference used to be
    public static final double kDriveMaxSpeed = 1.0;
    public static final double kModuleMaxAngularVelocity = DriveTrainPID.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public final CANSparkMax m_driveMotor;
    public final CANSparkMax m_turningMotor;

    private final SparkPIDController m_drivePID;

    public final RelativeEncoder m_driveEncoder;
    private final DutyCycleEncoder m_turningEncoder;


    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel     CAN ID for the drive motor.
     * @param turningMotorChannel   CAN ID for the turning motor.
     * @param driveEncoder          DIO input for the drive encoder channel A
     * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
     * @param turnOffset            offset from 0 to 1 for the home position of the
     *                              encoder
     */
    /*
     * private void intizialze(){
     * m_turningPIDController.reset(new
     * TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
     * kModuleMaxAngularAcceleration));
     * }
     */
    @Override
    public void periodic() {
        // m_turningEncoder.getCountsPerRevolution();
        double turnEncVal =  Units.radiansToDegrees(m_turningEncoder.getDistance());
        double driveEncPos = m_driveEncoder.getPosition();
        double driveEncVel = m_driveEncoder.getVelocity();

        SmartDashboard.putNumber("Robot/Swerve/Turn Encoder/ID: " + m_turningMotor.getDeviceId(), turnEncVal);
        SmartDashboard.putNumber("Robot/Swerve/Drive Encoder/ID: " + m_driveMotor.getDeviceId() + "/Pos" , driveEncPos);
        SmartDashboard.putNumber("Robot/Swerve/Drive Encoder/ID: " + m_driveMotor.getDeviceId() + "/Vel" , driveEncVel);
        super.periodic();
    }

    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turnEncoderPWMChannel, double turnOffset) {
        // can spark max motor controller objects
        m_driveMotor = new CANSparkMax(driveMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();

        m_turningMotor = new CANSparkMax(turningMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
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
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        // the getVelocity() function normally returns RPM but is scaled in the
        // SwerveModule constructor to return actual wheel speed
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            Rotation2d.fromRadians(m_turningEncoder.getDistance())
        );
    }

    public SwerveModulePosition getModulePosition() { // TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            Rotation2d.fromRadians(m_turningEncoder.getDistance())
        );
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState,
            getModulePosition().angle
        );

        // Calculate the drive output from the drive PID controller.
        // final double driveOutput = m_drivePIDController.calculate(
        // m_driveEncoder.getVelocity(), state.speedMetersPerSecond );

        final double signedAngleDifference = closestAngleCalculator(
            getModulePosition().angle.getRadians(),
            optimizedState.angle.getRadians()
        );
        double rotateMotorPercentPower = signedAngleDifference / (2 * Math.PI); // proportion error control //2

        m_driveMotor.set(optimizedState.speedMetersPerSecond / kDriveMaxSpeed);
        m_turningMotor.set(1.6 * rotateMotorPercentPower);
    }

    /**
     * Calculates the closest angle and direction between two points on a circle.
     * 
     * @param currentAngle
     *                     <ul>
     *                     <li>where you currently are
     *                     </ul>
     *                     </li>
     * @param desiredAngle
     *                     <ul>
     *                     <li>where you want to end up
     *                     </ul>
     *                     </li>
     * @return
     *         <ul>
     *         <li>signed double of the angle (rad) between the two points
     *         </ul>
     *         </li>
     */
    public double closestAngleCalculator(double currentAngle, double desiredAngle) {
        double signedDiff = 0.0;
        double rawDiff = currentAngle > desiredAngle ? currentAngle - desiredAngle : desiredAngle - currentAngle; // find
                                                                                                                  // the
                                                                                                                  // positive
                                                                                                                  // raw
                                                                                                                  // distance
                                                                                                                  // between
                                                                                                                  // the
                                                                                                                  // angles
        double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle

        if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
            signedDiff = ((2 * Math.PI) - modDiff); // full circle minus the angle
            if (desiredAngle > currentAngle)
                signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
        } else {
            signedDiff = modDiff;
            if (currentAngle > desiredAngle)
                signedDiff = signedDiff * -1;
        }
        return signedDiff;
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }
}
