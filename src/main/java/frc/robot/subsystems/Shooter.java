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
    private CANSparkMax shooterMtrLeft = new CANSparkMax(Constants.shooterMtrLeftC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax shooterMtrRight = new CANSparkMax(Constants.shooterMtrRightC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax ShooterAngleMtr = new CANSparkMax(Constants.shooterAngleMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder shooterMtrLeftEnc = shooterMtrLeft.getEncoder();
    private RelativeEncoder shooterMtrRightEnc = shooterMtrRight.getEncoder();
    private RelativeEncoder AngleMtrEnc = ShooterAngleMtr.getEncoder();

    private ProfiledPIDController m_angleController = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(0.05, 1.0));

    // Constants to calculate the angle of the shooter
    private static double LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET = Units.inchesToMeters(5.4375);
    private static double LENGTH_OF_SHOOTER_LINK = Units.inchesToMeters(3.9453125);
    private static double LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK = Units.inchesToMeters(9.6796875);
    private static double MOTOR_ANGLE_GEAR_RATIO = 5.0;
    private static double LEAD_SCREW_PITCH = 1.0; // TODO: radians to meters
    private static double HEIGHT_OFFSET_LEADSCREW = Units.inchesToMeters(6.375);

    public Shooter() {
        // shooterMtrLeft.follow(shooterMtrRight, true);
        double positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO;
        AngleMtrEnc.setPositionConversionFactor(positionConversionFactor);
        AngleMtrEnc.setVelocityConversionFactor(positionConversionFactor / 60);

        m_angleController.setTolerance(Units.degreesToRadians(0.5));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter/Motor Speed Left", Units.rotationsPerMinuteToRadiansPerSecond(shooterMtrLeftEnc.getVelocity()));
        SmartDashboard.putNumber("Shooter/Motor Speed Right", Units.rotationsPerMinuteToRadiansPerSecond(shooterMtrRightEnc.getVelocity()));
        SmartDashboard.putNumber("Shooter/Lead Screw Travel", AngleMtrEnc.getPosition());
        SmartDashboard.putNumber("Shooter/Aiming Angle", GetShooterAngle().getDegrees());
        
    }

    public Command RunShooter() {
        double speed = 0.85; // percent
        return runOnce(
                () -> {
                    SetShooterAngleSpeed(speed);
                });
    }

    public Command StopShooter() {
        return run(
                () -> {
                    shooterMtrLeft.set(0);
                    shooterMtrRight.set(0);
                });
    }

    public Command AngleUpShooter() {
        return run(
                () -> {
                    ShooterAngleMtr.set(.95);
                });
    }

    public Command AngleDownShooter() {
        return run(
                () -> {
                    ShooterAngleMtr.set(-.95);
                });
    }

    public void SetShooterAngleSpeed(double maxSpeedPercent)
    {
        shooterMtrLeft.set(-maxSpeedPercent);
        shooterMtrRight.set(maxSpeedPercent);
    }

    public Command AngleStop() {
        return run(
                () -> {
                    ShooterAngleMtr.set(0);
                });
    }

    public Rotation2d GetShooterAngle() {
        double heightOfLeadScrew = GetHeightAlongLeadScrew();

        // geometrical equations
        double interiorLength = Math.sqrt(
            heightOfLeadScrew * heightOfLeadScrew + LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET * LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET
        );
        double interiorAngle = Math.atan2(heightOfLeadScrew, LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET);
        double exteriorAngle = Math.acos(
            (
                LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK
                + interiorLength * interiorLength
                - LENGTH_OF_SHOOTER_LINK * LENGTH_OF_SHOOTER_LINK
            ) /
            (
                2 * LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK * interiorLength
            )
        );

        double shooterAngle = interiorAngle + exteriorAngle;

        return Rotation2d.fromRadians(shooterAngle);

    }

    public double GetHeightAlongLeadScrew()
    {
        return AngleMtrEnc.getPosition() + HEIGHT_OFFSET_LEADSCREW;
    }

}
