package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class IntakeWheels extends SubsystemBase {
    // tells which state the intake is in currently
    public enum State {
        PositionUp,
        PositionDown,
        PositionAmp,
        PositionOther
    }

    private CANSparkMax intakeWheelMtrR = new CANSparkMax(Constants.Intake.WheelMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    private State m_CurrentState = State.PositionUp;

    // TODO WILL ONLY BE 1 WHEEL MTR not 2!!
    // public int IntakePos = 1;
    public static final double GEAR_RATIO = 30.0; // TODO: TARGET ANGLE IN DEGREES OF THE MOTOR
    // I set this at 410 to account for gravity orginal value was 445 -Ben
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(i2cPort);
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
    public IntakeWheels() {

    }

    @Override
    public void periodic() {
        double IR = m_colorSensorV3.getIR();
        SmartDashboard.putNumber("Intake/Color Sensor/IR", IR);
        SmartDashboard.putBoolean("Intake/Note is Loaded", NoteIsLoaded());
    }

    public State GetIntakeState() {
        return m_CurrentState;
    }

    public boolean NoteIsLoaded() {
        return m_colorSensorV3.getIR() >= 6;
    }

    public Command RunIntakeWheels() {
        return run(
                () -> {
                    if (NoteIsLoaded()) {
                        intakeWheelMtrR.set(0);
                    } else {
                        intakeWheelMtrR.set(-0.75);
                    }

                });
    }

    public Command StopIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtrR.set(0);
                });
    }

    public Command ReverseIntakeWheels() {
        return runOnce(
                () -> {
                    intakeWheelMtrR.set(0.95);
                });
    }

}
