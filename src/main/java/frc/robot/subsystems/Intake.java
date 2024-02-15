package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {

    public CANSparkMax intakeAngleMtr = new CANSparkMax(Constants.intakeAngleMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax intakeWheelMtr1 = new CANSparkMax(Constants.intakeWheelMtr1C,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax intakeWheelMtr2 = new CANSparkMax(Constants.intakeWheelMtr2C,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private static final double intakeInteriorAngleRadians = Units.degreesToRadians(30);
    public RelativeEncoder intakeAngleMtrEnc = intakeAngleMtr.getEncoder();
    public RelativeEncoder intakeWheelMtr1Enc = intakeWheelMtr1.getEncoder();
    public RelativeEncoder intakeWheelMtr2Enc = intakeWheelMtr2.getEncoder();

    public ArmFeedforward IntakeFeedForward = new ArmFeedforward(0, 0, 0);
    public PIDController m_ArmController = new PIDController(0, 0, 0);

    public Intake() {
        // intakeWheelMtr1.follow(intakeWheelMtr2);
        double intakeAngleMotorPositionConversion = 2 * Math.PI; // revolutions -> radians
        double intakeAngleMotorVelocityConversion = intakeAngleMotorPositionConversion / 60; // rpm -> radians/second
        intakeAngleMtrEnc.setPositionConversionFactor(intakeAngleMotorPositionConversion);
        intakeAngleMtrEnc.setVelocityConversionFactor(intakeAngleMotorVelocityConversion);

        intakeWheelMtr2.follow(intakeWheelMtr1, true); // pair the intake wheels
    }

    @Override
    public void periodic() {
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Robot/Intake/Position", getIntakeAngle().getDegrees());
        SmartDashboard.putNumber("Robot/Intake/Encoder/Position", getEncoderAngle().getDegrees());
    }

    public Command RaiseIntake() {
        return run(
                () -> {
                    intakeAngleMtr.set(.5);
                });
    }

    public Command LowerIntake() {
        return runOnce(
                () -> {
                    intakeAngleMtr.set(-.5);
                });
    }

    public Command StopIntake() {
        return runOnce(
                () -> {
                    intakeAngleMtr.set(0);
                });
    }

    public Command RunIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtr1.set(.5);
                });
    }

    public Command StopIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtr1.set(0);
                });
    }

    public Command ReverseIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtr1.set(1);
                });
    }

    public void setIntakeAngle(Rotation2d angle) {
        double IntakeAngleFeedforward = IntakeFeedForward.calculate(angle.getRadians(), 0);
        double IntakeAngleFeedback = m_ArmController.calculate(getIntakeAngle().getRadians(), angle.getRadians());
        intakeAngleMtr.setVoltage(IntakeAngleFeedback + IntakeAngleFeedforward); // TODO INCORRECT
    }

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromRadians(intakeAngleMtrEnc.getPosition());
    }

    public Rotation2d getIntakeAngle() {
        return getEncoderAngle().plus(Rotation2d.fromRadians(intakeInteriorAngleRadians));
    }

}
