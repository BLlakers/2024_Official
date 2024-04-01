package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  private HangerModule leftHanger;
  private HangerModule rightHanger;

  double hangSpeedUp = 0.75;
  double hangSpeedDown = -0.75;

  public Hanger() { // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
    setName("Hanger");
    leftHanger =
        new HangerModule(
            Constants.Hanger.LeftMtrC,
            CANSparkLowLevel.MotorType.kBrushless,
            Constants.Port.hangerLeftMagSwitchDIOC);
    rightHanger =
        new HangerModule(
            Constants.Hanger.RightMtrC,
            CANSparkLowLevel.MotorType.kBrushless,
            Constants.Port.hangerRightMagSwitchDIOC);
  }

  public Command ResetHangCmd() {
    return this.runOnce(this::ResetHangEnc);
  }

  public void LeftHangUp() {
    leftHanger.MoveHang(hangSpeedUp);
  }

  public void LeftHangDown() {
    leftHanger.MoveHang(hangSpeedDown);
  }

  public void LeftHangStop() {
    leftHanger.MoveHang(0);
  }

  public void HangStop() {
    leftHanger.MoveHang(0);
    rightHanger.MoveHang(0);
  }

  public Command HangStopCommand() {
    return this.runOnce(this::HangStop);
  }

  public void RightHangUp() {
    rightHanger.MoveHang(hangSpeedUp);
  }

  public void RightHangDown() {
    rightHanger.MoveHang(hangSpeedDown);
  }

  public void RightHangStop() {
    rightHanger.MoveHang(0);
  }

  public boolean LeftHangIsDown() {
    return leftHanger.HangIsDown();
  }

  public boolean RightHangIsDown() {
    return rightHanger.HangIsDown();
  }

  public double GetLeftPosition() {
    return leftHanger.GetPosition();
  }

  public double GetRightPosition() {
    return rightHanger.GetPosition();
  }

  public void ResetHangEnc() {
    leftHanger.ResetHangEnc();
    rightHanger.ResetHangEnc();
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

    builder.addDoubleProperty("Left/Hang Pos", leftHanger.hangerMtrEnc::getPosition, null);
    builder.addDoubleProperty("Right/Hang Pos", rightHanger.hangerMtrEnc::getPosition, null);
    builder.addDoubleProperty("Left/Current", leftHanger.hangerMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Right/Current", rightHanger.hangerMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Left/Hang Velo", leftHanger.hangerMtrEnc::getVelocity, null);
    builder.addDoubleProperty("Right/Hang Velo", rightHanger.hangerMtrEnc::getVelocity, null);
    builder.addBooleanProperty("Left/MagSwitch Engaged", this::LeftHangIsDown, null);
    builder.addBooleanProperty("Right/MagSwitch Engaged", this::RightHangIsDown, null);
  }
}
