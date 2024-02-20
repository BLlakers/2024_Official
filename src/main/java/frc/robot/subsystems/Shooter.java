package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.*;

import com.fasterxml.jackson.databind.node.POJONode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMtrLeft = new CANSparkMax(Constants.shooterMtrLeftC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_shooterMtrRight = new CANSparkMax(Constants.shooterMtrRightC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_shooterAngleMtr = new CANSparkMax(Constants.shooterAngleMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder m_shooterMtrLeftEnc = m_shooterMtrLeft.getEncoder();
    private RelativeEncoder m_shooterMtrRightEnc = m_shooterMtrRight.getEncoder();
    private RelativeEncoder m_angleMtrEnc = m_shooterAngleMtr.getEncoder();

    private DigitalInput m_limitSwitchTop = null;
    private DigitalInput m_limitSwitchBottom = null;

    // Constants to calculate the angle of the shooter
    private static final double LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET = Units.inchesToMeters(5.4375);
    private static final double LENGTH_OF_SHOOTER_LINK = Units.inchesToMeters(3.9453125);
    private static final double LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK = Units.inchesToMeters(9.6796875);
    private static final double MOTOR_ANGLE_GEAR_RATIO = 5.0;
    private static final double LEAD_SCREW_PITCH = 1.0; // TODO: revolutions to meters of travel conversion
    private static final double HEIGHT_OFFSET_LEADSCREW = Units.inchesToMeters(6.375);

    // Constants
    public static final double s_angleMotorSpeedPercentage = 0.95;
    public static final double s_positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO; // meters
    public static final double s_velocityConversionFactor = s_positionConversionFactor / 60; // meters per second
    public static final double s_maxAngleMotorSpeed = Constants.NeoMaxSpeedRPM * s_velocityConversionFactor ;

    public Shooter() {
        // shooterMtrLeft.follow(shooterMtrRight, true);
        double positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO;
        m_angleMtrEnc.setPositionConversionFactor(positionConversionFactor);
        m_angleMtrEnc.setVelocityConversionFactor(positionConversionFactor / 60);

        // limit switches
        if (Constants.shooterLimitSwitchTopDIO >= 0)
            m_limitSwitchTop = new DigitalInput(Constants.shooterLimitSwitchTopDIO);
        if (Constants.shooterLimitSwitchBottomDIO >= 0)
            m_limitSwitchBottom = new DigitalInput(Constants.shooterLimitSwitchBottomDIO);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter/Motor Speed Left",
                Units.rotationsPerMinuteToRadiansPerSecond(m_shooterMtrLeftEnc.getVelocity()));
        SmartDashboard.putNumber("Shooter/Motor Speed Right",
                Units.rotationsPerMinuteToRadiansPerSecond(m_shooterMtrRightEnc.getVelocity()));
        SmartDashboard.putNumber("Shooter/Lead Screw Travel", m_angleMtrEnc.getPosition());
        SmartDashboard.putNumber("Shooter/Aiming Angle", GetShooterAngle().getDegrees());

    }

    public void CalibrateShooterAngle()
    {
        // update to be non-blocking
        if (m_limitSwitchBottom == null)
            return; // don't calibrate if you don't have a limit switch
        while (!BottomLimitSwitchTripped())
            SetShooterAngleSpeedPercentage(-s_angleMotorSpeedPercentage);

        m_angleMtrEnc.setPosition(0);
    }

    public void Shoot()
    {
        double speed = 0.85; // percentage
        SetShootingSpeed(speed);
    }

    public Command RunShooter() {
        
        return runOnce(
                () -> {
                    Shoot();
                });
    }

    public Command StopShooter() {
        return run(
                () -> {
                    m_shooterMtrLeft.set(0);
                    m_shooterMtrRight.set(0);
                });
    }

    public Command AngleUpShooter() {
        return run(
                () -> {
                    SetShooterAngleSpeedPercentage(s_angleMotorSpeedPercentage);
                });
    }

    public Command AngleDownShooter() {
        return run(
                () -> {
                    SetShooterAngleSpeedPercentage(-s_angleMotorSpeedPercentage);
                });
    }

    /**
     * Shoot with a desired speed
     * 
     * @param maxSpeedPercent: (-1, 1) - the maximum speed percentage to run the
     *                         shooter motors
     */
    public void SetShootingSpeed(double maxSpeedPercent) {
        m_shooterMtrLeft.set(-maxSpeedPercent);
        m_shooterMtrRight.set(maxSpeedPercent);
    }

    /**
     * Set the speed for angling the shooter motor
     * 
     * @param maxSpeedPercent: (-1, 1) - the maximum speed percentage to run the
     *                         shooter motors
     */
    public void SetShooterAngleSpeedPercentage(double maxSpeedPercent) {
        if (maxSpeedPercent > 0 && TopLimitSwitchTripped()) {
            m_shooterAngleMtr.stopMotor();
            return;
        }

        else if (maxSpeedPercent < 0 && BottomLimitSwitchTripped()) {
            m_shooterAngleMtr.stopMotor();
            return;
        }

        m_shooterAngleMtr.set(maxSpeedPercent);
    }

    public void SetShooterAngleSpeed(double radiansPerSecond)
    {
        // TODO: drive the velocity given amount of rotational velocity we want to achieve
    }

    /**
     * Check if top limit switch has been tripped
     * 
     * @return true if the top limit switch is tripped. false otherwise
     */
    public boolean TopLimitSwitchTripped() {
        if (m_limitSwitchTop == null)
            return false;
        return m_limitSwitchTop.get();
    }

    /**
     * Check if top limit switch has been tripped
     * 
     * @return true if the top limit switch is tripped. false otherwise
     */
    public boolean BottomLimitSwitchTripped() {
        if (m_limitSwitchBottom == null)
            return false;
        return m_limitSwitchBottom.get();
    }

    /**
     * Stop the angle motor (used for bindings)
     * 
     * @return Command to stop angling the motor
     */
    public Command AngleStop() {
        return run(
                () -> {
                    AngleMotorStop();
                });
    }

    /**
     * Stop the angle motor
     */
    public void AngleMotorStop() {
        m_shooterAngleMtr.set(0);
    }

    /**
     * Get the current aiming angle of the shooter
     * 
     * @return Rotation2d object of the shooter's angle
     */
    public Rotation2d GetShooterAngle() {
        double heightOfLeadScrew = GetHeightAlongLeadScrew();

        // geometrical equations
        double interiorLength = Math.sqrt(
                heightOfLeadScrew * heightOfLeadScrew
                        + LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET * LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET);
        double interiorAngle = Math.atan2(heightOfLeadScrew, LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET); // bottom triangle
        double exteriorAngle = Math.acos( // top triangle LAW OF COSINES
                (LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK
                        + interiorLength * interiorLength
                        - LENGTH_OF_SHOOTER_LINK * LENGTH_OF_SHOOTER_LINK) /
                        (2 * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * interiorLength));

        double shooterAngle = interiorAngle + exteriorAngle; // all angle

        return Rotation2d.fromRadians(shooterAngle);

    }

    /**
     * Get the height of travel measured from the encoders including the offset
     * 
     * @return The height of the pushed up point of the shooter (in meters)
     */
    public double GetHeightAlongLeadScrew() {
        return m_angleMtrEnc.getPosition() + HEIGHT_OFFSET_LEADSCREW;
    }

}
