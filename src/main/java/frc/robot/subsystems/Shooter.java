package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
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
    private Stuff m_Stuff = new Stuff();

    // Constants to calculate the angle of the shooter
    private static double LEAD_SCREW_CONNECTOR_HORIZONTAL_OFFSET = Units.inchesToMeters(0); // TODO
    private static double LENGTH_OF_SHOOTER_LINK = Units.inchesToMeters(0.0); // TODO
    private static double LENGTH_BETWEEN_SHOOTER_BASE_AND_LINK = Units.inchesToMeters(0.0); // TODO
    private static double MOTOR_ANGLE_GEAR_RATIO = 1.0; // TODO
    private static double LEAD_SCREW_PITCH = 1.0; // TODO: radians to meters
    private static double HEIGHT_OFFSET_LEADSCREW = 0.0; // TODO: meters

    public Shooter() {
        // shooterMtrLeft.follow(shooterMtrRight, true);
        double positionConversionFactor = LEAD_SCREW_PITCH / MOTOR_ANGLE_GEAR_RATIO;
        AngleMtrEnc.setPositionConversionFactor(positionConversionFactor);
        AngleMtrEnc.setVelocityConversionFactor(positionConversionFactor / 60);
    }

    @Override

    public void periodic() {
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMtrLeftEnc.getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Angle", AngleMtrEnc.getPosition());
    }

    public Command RunShooter() {
        return runOnce(
                () -> {
                    shooterMtrLeft.set(-.85);
                    shooterMtrRight.set(.85);
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
