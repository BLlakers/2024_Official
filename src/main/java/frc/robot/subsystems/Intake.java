package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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

    private State m_CurrentState = State.PositionUp;

    // TODO WILL ONLY BE 1 WHEEL MTR not 2!!
    public RelativeEncoder intakeAngleMtrEnc = intakeAngleMtr.getEncoder();
    // public int IntakePos = 1;
    public static final double GEAR_RATIO = 30.0; // TODO: TARGET ANGLE IN DEGREES OF THE MOTOR
    
    public static final double PosDownAngle = 68; // Down
    public static final double PosUpAngle = 0; // starting
    public static final double PosAmpAngle = 30; // This needs to be measured TODO
    public static final double PositionDown = 60;
    public static final double PositionUp = 20;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    public Intake() {
        // intakeWheelMtr1.follow(intakeWheelMtr2);
        double intakeAngleMotorPositionConversion = 2 * Math.PI / Intake.GEAR_RATIO; // revolutions -> radians
        double intakeAngleMotorVelocityConversion = intakeAngleMotorPositionConversion / 60; // rpm -> radians/second
        intakeAngleMtrEnc.setPositionConversionFactor(intakeAngleMotorPositionConversion);
        intakeAngleMtrEnc.setVelocityConversionFactor(intakeAngleMotorVelocityConversion);
        resetIntakeAngle();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Angle", GetIntakeMotorAngle().getDegrees());
        SmartDashboard.putNumber("Intake/Motor/Position", intakeAngleMtrEnc.getPosition());

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


    public Command RaiseIntake() {
        return run(
                () -> {
                    intakeAngleMtr.set(-.35);

                });
    }

    public Command autoIntakeUp() {
        return run(
                () -> {
                    if (GetIntakeMotorAngle().getDegrees() <= 30) {
                        intakeAngleMtr.set(0);
                    } else {
                        intakeAngleMtr.set(-.25);
                    }
                });
    }

    public Command autoAmp() {
        return run(() -> {
            double angleLo = 67.5;
            double angleHi = 72.5;
            
            double angle = GetIntakeMotorAngle().getDegrees();
            if (angle < angleLo) {
                intakeAngleMtr.set(0.25);
            } else if (angle > angleHi) {
                intakeAngleMtr.set(-0.25);
            } else if (angle > angleLo && angle < angleHi) {
                intakeAngleMtr.set(0);

            }
        }); // pos amp
    }

    public Command autoIntakeDown() {
        return run(
                () -> {
                    if (GetIntakeMotorAngle().getDegrees() >= 90) {
                        intakeAngleMtr.set(0);
                    } else{
                        intakeAngleMtr.set(0.25);
                    }
                });
    }

    public Command LowerIntake() {
        return runOnce(
                () -> {
                    intakeAngleMtr.set(0.35);
                });
    }

    public Command StopIntake() {
        return runOnce(
                () -> {
                    intakeAngleMtr.set(0);
                });
    }

    public Rotation2d GetIntakeMotorAngle() {
        return Rotation2d.fromRadians(intakeAngleMtrEnc.getPosition());
    }

    public void resetIntakeAngle() {
        intakeAngleMtrEnc.setPosition(0);
    }

    public Command resetIntakePos() {
        return runOnce(
                () -> {
                    resetIntakeAngle();
                });
    }

}
