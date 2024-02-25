package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class Intake extends SubsystemBase {
    // tells which state the intake is in currently
    public enum State {
        PositionUp,
        PositionDown,
        PositionAmp,
        PositionOther
    }

    private CANSparkMax intakeAngleMtr = new CANSparkMax(Constants.Intake.AngleMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax intakeWheelMtr = new CANSparkMax(Constants.Intake.IntakeWheelMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    

    private State m_CurrentState = State.PositionUp;

    // TODO WILL ONLY BE 1 WHEEL MTR not 2!!
    private RelativeEncoder intakeAngleMtrEnc = intakeAngleMtr.getEncoder();
    private RelativeEncoder intakeWheelMtr1Enc = intakeWheelMtr.getEncoder();

    // public int IntakePos = 1;
    public static final double GEAR_RATIO = 100.0; // TODO: TARGET ANGLE IN DEGREES OF THE MOTOR
    // I set this at 410 to account for gravity orginal value was 445 -Ben

    public static final double PosDownAngle = 68; // Down
    public static final double PosUpAngle = 0; // starting
    public static final double PosAmpAngle = 30; // This needs to be measured TODO
    public Intake() {
        double intakeAngleMotorPositionConversion = 2 * Math.PI; // revolutions -> radians
        double intakeAngleMotorVelocityConversion = intakeAngleMotorPositionConversion / 60; // rpm -> radians/second
        intakeAngleMtrEnc.setPositionConversionFactor(intakeAngleMotorPositionConversion);
        intakeAngleMtrEnc.setVelocityConversionFactor(intakeAngleMotorVelocityConversion);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Position", GetIntakeMotorAngle().getDegrees());

        if (GetIntakeMotorAngle().getDegrees() >= Intake.PosUpAngle) {
            m_CurrentState = State.PositionUp;
        } else if (GetIntakeMotorAngle().getDegrees() <= Intake.PosDownAngle) {
            m_CurrentState = State.PositionDown;
        } else if (Math.abs(GetIntakeMotorAngle().getDegrees() - Intake.PosAmpAngle) <= 1)
            m_CurrentState = State.PositionAmp;

        else
            m_CurrentState = State.PositionOther;
    }

    public State GetIntakeState() {
        return m_CurrentState;
    }

    public boolean NoteIsLoaded() {
        return false; // TODO
    }

    public Command LowerIntake() {
        return run(
                () -> {
                    intakeAngleMtr.set(.17);
                });
    }

    public Command RaiseIntake() {
        return runOnce(
                () -> {
                    intakeAngleMtr.set(-.45);
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
                    intakeWheelMtr.set(-.5); // inverted
                });
    }

    public Command StopIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtr.set(0);
                });
    }

    public Command ReverseIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtr.set(1);
                });
    }

    public Rotation2d GetIntakeMotorAngle() {
        return Rotation2d.fromRadians(intakeAngleMtrEnc.getPosition());
    }
}
