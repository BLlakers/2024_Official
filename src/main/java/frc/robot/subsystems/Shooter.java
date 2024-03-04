package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMtrLeft = new CANSparkMax(Constants.Shooter.LeftMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_shooterMtrRight = new CANSparkMax(Constants.Shooter.RightMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_shooterAngleMtr = new CANSparkMax(Constants.Shooter.AngleMtrC,
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
    private static final double HEIGHT_OF_SHOOTER_BASE = Units.inchesToMeters(9.0);
    private static final double HEIGHT_OFFSET_LEADSCREW = Units.inchesToMeters(6.375);

    // Constants
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(55);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(30);
    public static final double s_angleMotorSpeedPercentage = 0.1;
    public static final double s_positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO; // meters
    public static final double s_velocityConversionFactor = s_positionConversionFactor / 60; // meters per second
    public static final double s_maxAngleMotorSpeed = Constants.Conversion.NeoMaxSpeedRPM * s_velocityConversionFactor;

    private static double s_LeftMotorShooterSpeed = 0.85;
    private static double s_RightMotorShooterSpeed = -0.85;

    public Shooter() {
        setName("Shooter");

        // shooterMtrLeft.follow(shooterMtrRight, true);
        double positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO;
        m_angleMtrEnc.setPositionConversionFactor(positionConversionFactor);
        m_angleMtrEnc.setVelocityConversionFactor(positionConversionFactor / 60);

        // limit switches
        m_limitSwitchTop = new DigitalInput(Constants.Shooter.LimitSwitchTopDIO);
        // m_limitSwitchBottom = new
        // DigitalInput(Constants.Shooter.LimitSwitchBottomDIO);

    }

    public Command CalibrateShooterAngle() {
        throw new UnsupportedOperationException("This is untested code. Wait for Dimitri");
        // return this.AngleUpShooter()
        //         .finallyDo(() -> {
        //             m_angleMtrEnc.setPosition(0);
        //         });
    }

    public void Shoot() {
        SetShootingSpeed(s_LeftMotorShooterSpeed, s_RightMotorShooterSpeed);
    }

    public Command RunShooter() {

        return this.runEnd(this::Shoot, this::StopShooter);
    }

    public Command StopShooterCommand() {
        return this.runOnce(this::StopShooter);
    }

    public void StopShooter() {

        m_shooterMtrLeft.stopMotor();
        m_shooterMtrRight.stopMotor();
    }

    public Command AngleUpShooter() {
        Command cmd = this.run(() -> {
            SetShooterAngleSpeedPercentage(s_angleMotorSpeedPercentage);
        });

        ConditionalCommand cmdWithLimit = cmd.unless(this::TopLimitSwitchTripped);
        return cmdWithLimit.finallyDo(this::AngleMotorStop);
    }

    public Command AngleDownShooter() {
        Command cmd = this.run(() -> {
            SetShooterAngleSpeedPercentage(-s_angleMotorSpeedPercentage);
        });

        ConditionalCommand cmdWithLimit = cmd.unless(this::BottomLimitSwitchTripped);

        return cmdWithLimit.finallyDo(this::AngleMotorStop);
    }

    /**
     * Shoot with a desired speed
     * 
     * @param maxSpeedPercent: (-1, 1) - the maximum speed percentage to run the
     *                         shooter motors
     */
    public void SetShootingSpeed(double maxSpeedPercent) {
        SetShootingSpeed(maxSpeedPercent, -maxSpeedPercent);
    }

    public void SetShootingSpeed(double maxSpeedPercentLeft, double maxSpeedPercentRight) {
        m_shooterMtrLeft.set(maxSpeedPercentLeft);
        m_shooterMtrRight.set(maxSpeedPercentRight);
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

    public void SetShooterAngleSpeed(double radiansPerSecond) {
        // TODO: drive the velocity given amount of rotational velocity we want to
        // achieve
        throw new UnsupportedOperationException("Shooter.SetShooterAngleSpeed is not yet implemented.");
    }

    /**
     * Check if top limit switch has been tripped
     * 
     * @return true if the top limit switch is tripped. false otherwise
     */
    public boolean TopLimitSwitchTripped() {
        if (m_limitSwitchTop == null)
            return false;
        boolean tripped = m_limitSwitchTop.get();

        if (tripped)
            m_angleMtrEnc.setPosition(0); // calibrate the position
        return tripped;
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
        return this.runOnce(this::AngleMotorStop);
    }

    public Command ManualAngleUp() {
        return this.runEnd(
                () -> {
                    m_shooterAngleMtr.set(s_angleMotorSpeedPercentage);
                },
                this::AngleMotorStop);
    }

    public Command ManualAngleDown() {
        return this.runEnd(
                () -> {
                    m_shooterAngleMtr.set(-s_angleMotorSpeedPercentage);
                },
                this::AngleMotorStop);
    }

    /**
     * Stop the angle motor
     */
    public void AngleMotorStop() {
        m_shooterAngleMtr.stopMotor();
    }

    /**
     * Get the current aiming angle of the shooter
     * 
     * @return Rotation2d object of the shooter's angle
     */
    public Rotation2d GetShooterAngle() {
        double heightOfLeadScrew = GetHeightAlongLeadScrew();
        double heightOffsetOfShooterBase = HEIGHT_OF_SHOOTER_BASE - heightOfLeadScrew;

        // geometrical equations
        double interiorLength = Math.hypot(heightOffsetOfShooterBase, LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET);
        double interiorAngle = Math.atan2(heightOffsetOfShooterBase, LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET); // bottom
                                                                                                              // triangle
        double exteriorAngle = Math.acos( // LAW OF COSINES
                (LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK
                        + interiorLength * interiorLength
                        - LENGTH_OF_SHOOTER_LINK * LENGTH_OF_SHOOTER_LINK) /
                        (2 * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * interiorLength));

        double shooterAngle = exteriorAngle - interiorAngle;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Motor Left/Speed", m_shooterMtrLeftEnc::getVelocity, null);
        builder.addDoubleProperty("Motor Left/Speed Percentage",
                () -> m_shooterMtrLeftEnc.getVelocity() / Constants.Conversion.NeoMaxSpeedRPM, null);
        builder.addDoubleProperty("Motor Right/Speed", m_shooterMtrRightEnc::getVelocity, null);
        builder.addDoubleProperty("Motor Right/Speed Percentage",
                () -> m_shooterMtrLeftEnc.getVelocity() / Constants.Conversion.NeoMaxSpeedRPM, null);
        builder.addDoubleProperty("Angle Motor/Encoder/Position", m_angleMtrEnc::getPosition, null);
        builder.addDoubleProperty("Aiming Angle", () -> this.GetShooterAngle().getDegrees(), null);

        builder.addDoubleProperty("Motor Left/Speed Setpoint", () -> s_LeftMotorShooterSpeed,
                (double s) -> s_LeftMotorShooterSpeed = s);
        builder.addDoubleProperty("Motor Right/Speed Setpoint", () -> s_RightMotorShooterSpeed,
                (double s) -> s_RightMotorShooterSpeed = s);

        builder.addBooleanProperty("Top Limit Switch/Tripped", this::TopLimitSwitchTripped, null);
        builder.addBooleanProperty("Bottom Limit Switch/Tripped", this::BottomLimitSwitchTripped, null);
    }

}
