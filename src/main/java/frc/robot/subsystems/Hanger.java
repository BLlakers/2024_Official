package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  private HangerModule m_leftHanger;
  private HangerModule m_rightHanger;

  public static final double hangSpeedUp = 0.75;
  public static final double hangSpeedDown = -0.75;

  public Hanger() { // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
    setName("Hanger");
    m_leftHanger =
        new HangerModule(
            Constants.Hanger.LeftMtrC,
            CANSparkLowLevel.MotorType.kBrushless,
            Constants.Port.hangerLeftMagSwitchDIOC);
    m_rightHanger =
        new HangerModule(
            Constants.Hanger.RightMtrC,
            CANSparkLowLevel.MotorType.kBrushless,
            Constants.Port.hangerRightMagSwitchDIOC);
  }

  public Command ResetHangCmd() {
    return this.runOnce(this::ResetHangEnc);
  }

  public void StopHangers() {
    leftHangerModule().HangStop();
    rightHangerModule().HangStop();
  }

  public Command HangStopCommand() {
    return this.runOnce(this::StopHangers);
  }

  public void ResetHangEnc() {
    leftHangerModule().ResetHangEnc();
    rightHangerModule().ResetHangEnc();
  }

  public HangerModule leftHangerModule() {
    return m_leftHanger;
  }

  public HangerModule rightHangerModule() {
    return m_rightHanger;
  }

  /** Raises hang when Held. Will stop at top position */
  public Command RaiseHangAuto() {
    Command cmd =
        Commands.parallel(
                rightHangerModule()
                    .runEnd(rightHangerModule()::MoveHangUp, rightHangerModule()::HangStop)
                    .until(rightHangerModule()::HangIsAtPosition),
                leftHangerModule()
                    .runEnd(leftHangerModule()::MoveHangUp, leftHangerModule()::HangStop)
                    .until(leftHangerModule()::HangIsAtPosition))
            .finallyDo(this::StopHangers);

    cmd.addRequirements(this);

    return cmd;
  }

  /** Lowers hang when Held. Will stop when it hits the limit switch */
  public Command LowerHangAuto() {
    Command cmd =
        Commands.parallel(
                rightHangerModule()
                    .runEnd(rightHangerModule()::MoveHangDown, rightHangerModule()::HangStop)
                    .until(rightHangerModule()::HangIsDown),
                leftHangerModule()
                    .runEnd(leftHangerModule()::MoveHangDown, leftHangerModule()::HangStop)
                    .until(leftHangerModule()::HangIsDown))
            .finallyDo(this::StopHangers);

    cmd.addRequirements(this);

    return cmd;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    leftHangerModule().initSendable(builder);
    rightHangerModule().initSendable(builder);
  }
}
