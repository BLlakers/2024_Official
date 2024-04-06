package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class HangerModule extends SubsystemBase {
  private DigitalInput hangerMagSwitch;
  private CANSparkMax hangerMtr;
  private RelativeEncoder hangerMtrEnc;

  public HangerModule(
      int hangMtrPort,
      com.revrobotics.CANSparkLowLevel.MotorType MotorType,
      int hangLimitSwitchPort) {
    hangerMtr = new CANSparkMax(hangMtrPort, MotorType);
    hangerMtrEnc = hangerMtr.getEncoder();
    hangerMagSwitch = new DigitalInput(hangLimitSwitchPort);
    ResetHangEnc();
  }

  public void ResetHangEnc() {
    hangerMtrEnc.setPosition(0);
  }

  public Command ResetHangCmd() {
    return this.runOnce(this::ResetHangEnc);
  }

  public void MoveHang(double speed) {
    hangerMtr.set(speed);
  }

  public void MoveHangUp() {
    MoveHang(Hanger.s_hangSpeedUp);
  }

  public void MoveHangDown() {
    MoveHang(Hanger.s_hangSpeedDown);
  }

  public void HangStop() {
    hangerMtr.set(0);
  }

  public Command HangStopCommand() {
    return this.runOnce(this::HangStop);
  }

  public boolean HangIsDown() {
    return !hangerMagSwitch.get();
  }

  public double GetPosition() {
    return hangerMtrEnc.getPosition();
  }

  public boolean HangIsAtPosition() {
    if (GetPosition() >= 140) {
      return true;
    } else {
      return false;
    }
  }

  /** Raises hang when Held. Will stop at top position */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Hang Pos", hangerMtrEnc::getPosition, null);
    builder.addDoubleProperty("Current", hangerMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Left/Hang Velo", hangerMtrEnc::getVelocity, null);
    builder.addBooleanProperty("Left/MagSwitch Engaged", this::HangIsDown, null);
  }

  public interface HangerModuleExplanation {}
}
