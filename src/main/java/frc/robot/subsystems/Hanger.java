package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {

  // Here, We create each of our two Hangers: One on the left, and one on the right. 
  // These will be initialized in the contructor, although this really doesnt matter. 
  private HangerModule m_leftHanger;
  private HangerModule m_rightHanger;

  // We create 2 varibles for our speeds
  public static final double s_hangSpeedUp = 0.75;
  public static final double s_hangSpeedDown = -0.75;

/** This is the Hanger Class. <p>
 *In the Hanger Class, we have our Multi-HangModule Functions and Commands Such as:
 *<ul>  
 *<p> {@link #RaiseHangAuto()} (Which Automatically Raises the Hang Via encoder values) 
 *<p> {@link #LowerHangAuto()} (Which Automatically Lowers the Hang until the Switchs are hit) </ul>
 *<p> We also Define other commands, like:
 *<ul> <p> {@link #HangStopCommand()} (which Stops Both Hanger Modules)
 *<p> {@link #ResetHangCmd()} (Which Resets the position on each of the two encoders) </ul>
 *In order to Access each individual hanger run the following Functions: <ul> 
 *<p> {@link #leftHangerModule()} for the Left Hanger 
 *<p> {@link #rightHangerModule()} for the Right hanger.
 * @see 
 */
  public Hanger() { 
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
  public static int Explanation(){
  return 2;
  }
}
