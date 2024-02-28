package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;

public class IntakeWheels extends SubsystemBase {
    private CANSparkMax intakeWheelMtrR = new CANSparkMax(Constants.Intake.WheelMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    public IntakeWheels() {
        setName("Intake Wheels");

    }

    public boolean NoteIsLoaded() {
        return m_colorSensorV3.getIR() >= 15;
    }

    public Command RunIntakeWheelsCommand() {
        return this.runEnd(this::RunIntakeWheels, this::StopIntakeWheels);
    }

    public void RunIntakeWheels() {
        if (NoteIsLoaded()) {
            intakeWheelMtrR.set(0);
        } else {
            intakeWheelMtrR.set(-0.75);
        }
    }

    public Command StopIntakeWheelsCommand() {
        return this.runOnce(this::StopIntakeWheels);
    }

    public void StopIntakeWheels() {
        intakeWheelMtrR.set(0);

    }

    public Command ReverseIntakeWheelsCommand() {
        return this.runEnd(this::ReverseIntakeWheels, this::StopIntakeWheels);
    }

    public void ReverseIntakeWheels()
    {
        intakeWheelMtrR.set(0.95);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);

        builder.addIntegerProperty("Color Sensor/IR", m_colorSensorV3::getIR, null);
        builder.addBooleanProperty("Note is Loaded", this::NoteIsLoaded, null);
    }

}
