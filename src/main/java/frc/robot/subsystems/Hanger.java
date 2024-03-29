package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Hanger extends SubsystemBase {
  private DigitalInput hangLeftMagSwitch = new DigitalInput(Constants.Port.hangerLeftMagSwitchDIOC);
  private DigitalInput hangRightMagSwitch =
      new DigitalInput(Constants.Port.hangerRightMagSwitchDIOC);
  private CANSparkMax hangerLeftMtr =
      new CANSparkMax(
          Constants.Hanger.LeftMtrC, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax hangerRightMtr =
      new CANSparkMax(
          Constants.Hanger.RightMtrC, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  private RelativeEncoder hangerLeftMtrEnc = hangerLeftMtr.getEncoder();
  private RelativeEncoder hangerRightMtrEnc = hangerRightMtr.getEncoder();

  double hangSpeedUp = 0.75;
  double hangSpeedDown = -0.75;

  public Hanger() { // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
    setName("Hanger");
    ResetHangEnc();
  }

  public void ResetHangEnc() {
    hangerLeftMtrEnc.setPosition(0.0);
    hangerRightMtrEnc.setPosition(0.0);
  }

  public Command ResetHangCmd() {
    return this.runOnce(this::ResetHangEnc);
  }

  public void LeftHangUp() {
    hangerLeftMtr.set(hangSpeedUp);
  }

  public void LeftHangDown() {
    hangerLeftMtr.set(hangSpeedDown);
  }

  public void LeftHangStop() {
    hangerLeftMtr.set(0);
  }

  public void HangStop() {
    hangerLeftMtr.set(0);
    hangerRightMtr.set(0);
  }

  public Command HangStopCommand() {
    return this.runOnce(this::HangStop);
  }

  public void RightHangUp() {
    hangerRightMtr.set(hangSpeedUp);
  }

  public void RightHangDown() {
    hangerRightMtr.set(hangSpeedDown);
  }

  public void RightHangStop() {
    hangerRightMtr.set(0);
  }

  public boolean LeftHangIsDown() {
    return !hangLeftMagSwitch.get();
  }

  public boolean RightHangIsDown() {
    return !hangRightMagSwitch.get();
  }

  public double GetLeftPosition() {
    return hangerLeftMtrEnc.getPosition();
  }

  public double GetRightPosition() {
    return hangerRightMtrEnc.getPosition();
  }

  /** Raises hang when Held. Will stop at top position */
  public Command RaiseHangAuto() {
    Command cmd =
        Commands.parallel(
                Commands.runEnd(this::RightHangUp, this::RightHangStop)
                    .until(() -> this.GetRightPosition() >= 140),
                Commands.runEnd(this::LeftHangUp, this::LeftHangStop)
                    .until(() -> this.GetLeftPosition() >= 140))
            .finallyDo(this::HangStop);

    cmd.addRequirements(this);

    return cmd;
  }

  public boolean HangersDown() {
    if (RightHangIsDown() && LeftHangIsDown()) return true;

    return false;
  }

  /** Lowers hang when Held. Will stop when it hits the limit switch */
  public Command LowerHangAuto() {
    Command cmd =
        Commands.parallel(
                Commands.runEnd(this::RightHangDown, this::RightHangStop)
                    .until(this::RightHangIsDown),
                Commands.runEnd(this::LeftHangDown, this::LeftHangStop).until(this::LeftHangIsDown))
            .finallyDo(this::HangStop);

    cmd.addRequirements(this);

    return cmd;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Left/Hang Pos", hangerLeftMtrEnc::getPosition, null);
    builder.addDoubleProperty("Right/Hang Pos", hangerRightMtrEnc::getPosition, null);
    builder.addDoubleProperty("Left/Current", hangerLeftMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Right/Current", hangerRightMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Left/Hang Velo", hangerLeftMtrEnc::getVelocity, null);
    builder.addDoubleProperty("Right/Hang Velo", hangerRightMtrEnc::getVelocity, null);
    builder.addBooleanProperty("Left/MagSwitch Engaged", this::LeftHangIsDown, null);
    builder.addBooleanProperty("Right/MagSwitch Engaged", this::RightHangIsDown, null);
  }
}
