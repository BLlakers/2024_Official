package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;

public class IntakeWheels extends SubsystemBase {
  private CANSparkMax intakeWheelMtrR =
      new CANSparkMax(
          Constants.Intake.WheelMtrC, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(i2cPort);

  private static final double s_IntakeSpeed = 0.55;
  private static final double s_EjectSpeed = -0.95;
  private static final double s_ReIntakeNoteEjectSpeed = -0.4;
  private static final double s_IntakeOverrunSeconds = 0.1;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter. The device will be
   * automatically initialized with default parameters.
   */
  public IntakeWheels() {
    setName("Intake/Wheels");
  }

  public boolean NoteIsLoaded() {
    return m_colorSensorV3.getIR() >= 200;
  }

  public Command PowerOnIntakeWheelsCommand() {
    return this.runOnce(this::IntakeNote);
  }

  public Command IntakeNoteCommand() {
    return this.runEnd(this::IntakeNote, this::Stop).until(this::NoteIsLoaded);
  }

  public Command IntakeNoteCommandrunRegardless() {
    return this.runEnd(this::IntakeNote, this::Stop);
  }

  public Command IntakeNoteOverrunCommand() {
    return this.run(this::IntakeNote)
        .until(this::NoteIsLoaded)
        .andThen(Commands.waitSeconds(s_IntakeOverrunSeconds))
        .finallyDo(this::Stop);
  }

  public Command ReIntakeNoteCommand() {
    return Commands.repeatingSequence(
            this.run(
                    () -> {
                      intakeWheelMtrR.set(s_ReIntakeNoteEjectSpeed);
                    }) // partial eject
                .until(() -> m_colorSensorV3.getIR() < 250),
            this.run(this::IntakeNote) // re-intake
                .until(this::NoteIsLoaded))
        .finallyDo(this::Stop);
  }

  public void IntakeNote() {
    if (NoteIsLoaded()) {
      intakeWheelMtrR.set(0);
    } else {
      intakeWheelMtrR.set(s_IntakeSpeed);
    }
  }

  public Command StopCommand() {
    return this.runOnce(this::Stop);
  }

  public void Stop() {
    intakeWheelMtrR.set(0);
  }

  public Command PowerOnEjectIntakeWheelsCommand() {
    return this.runOnce(this::EjectNote);
  }

  public Command EjectNoteCommand() {
    return this.runEnd(this::EjectNote, this::Stop);
  }

  public void EjectNote() {
    intakeWheelMtrR.set(s_EjectSpeed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addIntegerProperty("Color Sensor/IR", m_colorSensorV3::getIR, null);
    builder.addIntegerProperty("Color Sensor/Blue", m_colorSensorV3::getBlue, null);
    builder.addIntegerProperty("Color Sensor/Red", m_colorSensorV3::getRed, null);
    builder.addIntegerProperty("Color Sensor/Green", m_colorSensorV3::getGreen, null);
    builder.addIntegerProperty("Color Sensor/Proximity", m_colorSensorV3::getProximity, null);
    builder.addBooleanProperty("Note/Is Loaded", this::NoteIsLoaded, null);
  }
}
