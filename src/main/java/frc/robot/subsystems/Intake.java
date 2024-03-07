package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {
  // tells which state the intake is in currently
  public enum State {
    PositionUp,
    PositionDown,
    PositionAmp,
    PositionOther;

    @Override
    public String toString() {
      Map<State, String> stateMap = new HashMap<State, String>();
      stateMap.put(State.PositionUp, "Position Up");
      stateMap.put(State.PositionDown, "Position Down");
      stateMap.put(State.PositionOther, "Position Other");
      stateMap.put(State.PositionAmp, "Position Amplifier");

      return stateMap.get(this);
    }
  }

  private CANSparkMax intakeAngleMtr =
      new CANSparkMax(
          Constants.Intake.AngleMtrC, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  private State m_CurrentState = State.PositionUp;

  private IntakeWheels m_IntakeWheels = new IntakeWheels();

  private static final double s_AngleDownStopDegrees = 90;
  private static final double s_AngleUpStopDegrees = 30;

  // TODO WILL ONLY BE 1 WHEEL MTR not 2!!
  public RelativeEncoder intakeAngleMtrEnc = intakeAngleMtr.getEncoder();
  // public int IntakePos = 1;
  public static final double GEAR_RATIO = 30.0; // TODO: TARGET ANGLE IN DEGREES OF THE MOTOR

  public static final Rotation2d PosDownAngle = Rotation2d.fromDegrees(140); // Down
  public static final Rotation2d PosUpAngle = Rotation2d.fromDegrees(5); // starting
  public static final Rotation2d PosAmpAngle = Rotation2d.fromDegrees(30); // This needs to be measured TODO
  public static final Rotation2d PositionDown = Rotation2d.fromDegrees(60);
  public static final Rotation2d PositionUp = Rotation2d.fromDegrees(20);

  private static final double s_IntakeAngleSpeedUp = -0.25;
  private static final double s_IntakeAngleSpeedDown = 0.25;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter. The device will be
   * automatically initialized with default parameters.
   */
  public Intake() {
    // intakeWheelMtr1.follow(intakeWheelMtr2);
    double intakeAngleMotorPositionConversion =
        2 * Math.PI / Intake.GEAR_RATIO; // revolutions -> radians
    double intakeAngleMotorVelocityConversion =
        intakeAngleMotorPositionConversion / 60; // rpm -> radians/second
    intakeAngleMtrEnc.setPositionConversionFactor(intakeAngleMotorPositionConversion);
    intakeAngleMtrEnc.setVelocityConversionFactor(intakeAngleMotorVelocityConversion);
    resetIntakeAngle();
    setName("Intake/Angle");
  }

  @Override
  public void periodic() {
    super.periodic();

    if (GetIntakeMotorAngle().getDegrees() <= Intake.PosUpAngle.getDegrees()) {
      m_CurrentState = State.PositionUp;
    } else if (GetIntakeMotorAngle().getDegrees() >= Intake.PosDownAngle.getDegrees()) {
      m_CurrentState = State.PositionDown;
    } else if (Math.abs(GetIntakeMotorAngle().getDegrees() - Intake.PosAmpAngle.getDegrees()) <= 1)
      m_CurrentState = State.PositionAmp;
    else m_CurrentState = State.PositionOther;
  }

  public IntakeWheels GetIntakeWheels() {
    return m_IntakeWheels;
  }

  public State GetIntakeState() {
    return m_CurrentState;
  }

  public Command ManualRaiseIntakeCommand() {
    return this.runEnd(this::RaiseIntake, this::StopIntake);
  }

  public void RaiseIntake() {
    intakeAngleMtr.set(s_IntakeAngleSpeedUp);
  }

  public boolean NoteIsLoaded() {
    return m_IntakeWheels.NoteIsLoaded();
  }

  public void MoveIntake(double speed)
  {
    intakeAngleMtr.set(speed);
  }

  public Command autoIntakeUp() {
    return this.run(this::RaiseIntake)
        .onlyWhile(() -> GetIntakeMotorAngle().getDegrees() > Intake.s_AngleUpStopDegrees)
        .finallyDo(this::StopIntake);
  }

  public Command autoAmp() {
    double angleLo = 67.5;
    double angleHi = 72.5;

    return this.runEnd(
        () -> {
          double angle = GetIntakeMotorAngle().getDegrees();
          if (angle < angleLo) {
            intakeAngleMtr.set(0.25);
          } else if (angle > angleHi) {
            intakeAngleMtr.set(-0.25);
          } else if (angle > angleLo && angle < angleHi) {
            intakeAngleMtr.set(0);
          }
        },
        this::StopIntake); // pos amp
  }

  public Command autoIntakeDown() {
    return this.run(this::LowerIntake)
        .onlyWhile(() -> GetIntakeMotorAngle().getDegrees() < Intake.s_AngleDownStopDegrees)
        .finallyDo(this::StopIntake);
  }

  public Command ManualLowerIntakeCommand() {
    return this.runEnd(this::LowerIntake, this::StopIntake);
  }

  public void LowerIntake() {
    intakeAngleMtr.set(s_IntakeAngleSpeedDown);
  }

  public Command StopIntakeCommand() {
    return this.runOnce(this::StopIntake);
  }

  public void StopIntake() {
    intakeAngleMtr.set(0);
  }

  public Rotation2d GetIntakeMotorAngle() {
    return Rotation2d.fromRadians(intakeAngleMtrEnc.getPosition());
  }

  public void resetIntakeAngle() {
    intakeAngleMtrEnc.setPosition(0);
  }

  public Command resetIntakePos() {
    return runOnce(this::resetIntakeAngle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    m_IntakeWheels.initSendable(builder);

    builder.addDoubleProperty("Angle", () -> this.GetIntakeMotorAngle().getDegrees(), null);
    builder.addDoubleProperty("Motor/Position", intakeAngleMtrEnc::getPosition, null);
    builder.addDoubleProperty("Motor/Velocity", intakeAngleMtrEnc::getVelocity, null);
    builder.addStringProperty("State", () -> this.GetIntakeState().toString(), null);
  }
}
