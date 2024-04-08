package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** This is the HangerModule class 
 * <p> This class is used to create individual HangerModules
 * <p> We do this because it is unneccesary to recreate The MotorController, Limit switch, and Encoder 2 times.
 */

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
/** This is a void function. <p> It sets the position on the encoder to 0 */
  public void ResetHangEnc() {
    hangerMtrEnc.setPosition(0);
  }
/** This is a command that runs the void function {@link #ResetHangEnc} */
  public Command ResetHangCmd() {
    return this.runOnce(this::ResetHangEnc);
  }
  /** This void function runs the Motor at what ever speed is provided.
   * @param speed (double)
   */
  public void MoveHang(double speed) {
    hangerMtr.set(speed);
  }
/** This void function runs {@link #MoveHang} and tells the Hanger to run at {@link Hanger#s_hangSpeedUp}
 * <li> This moves the Hanger up.
 */
  public void MoveHangUp() {
    MoveHang(Hanger.s_hangSpeedUp);
  }
/** This void function runs {@link #MoveHang} and tells the Hanger to run at {@link Hanger#s_hangSpeedDown}
 * <li> This moves the Hanger down
 */
  public void MoveHangDown() {
    MoveHang(Hanger.s_hangSpeedDown);
  }

/** This void function stops the Motor */
  public void HangStop() {
    hangerMtr.set(0);
  }
/** This is a command. It runs the void command {@link #HangStop} */
  public Command HangStopCommand() {
    return this.runOnce(this::HangStop);
  }
/** This function checks to see if the hang module is hitting the mag switch */
  public boolean HangIsDown() {
    return !hangerMagSwitch.get();
  }
/** This function sees what position the hanger is currently at in encoder counts (NOT CONVERTED) */
  public double GetPosition() {
    return hangerMtrEnc.getPosition();
  }
/** Says when the Hanger is at the top posistion
 * @return A boolean (If it is at the top position then it will return true)
*/
  public boolean HangIsAtPosition() {
    if (GetPosition() >= 140) {
      return true;
    } else {
      return false;
    }
  }
 Command AutoHangUp = runEnd(this::MoveHangUp, this::HangStop).until(this::HangIsAtPosition);
 
 Command AutoHangDown = runEnd(this::MoveHangDown, this::HangStop).until(this::HangIsDown);
 
  @Override

  /**Everytime we create a HangerModule object, the InitSendable will be dupcliated for that module. Same thing with SwerveModule objects. */
  
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Hang Pos", hangerMtrEnc::getPosition, null);
    builder.addDoubleProperty("Current", hangerMtr::getOutputCurrent, null);
    builder.addDoubleProperty("Left/Hang Velo", hangerMtrEnc::getVelocity, null);
    builder.addBooleanProperty("Left/MagSwitch Engaged", this::HangIsDown, null);
  }







  /** <b> DETAILED EXPLANATION </b> 
   * <p> In this class, we use void functions to run actions, and have commands run those void functions.
   * <p> Void functions don't occupy the command scheduler, but commands do. 
   * <p> If we wanted to indivdually run the module motor up while a button is held:
   * <p> We would run the Command.
   * <p> But, If we wanted to have a sequence of actions happening (like us running it until a certain point):
   * <p> We would run a set of void functions in a singular command, not held in this class (Would be in {@link Hanger}).
   * <p> EX: {@link Hanger#LowerHangAuto}
   * <p> In the Example, we run a set of void functions and boolean functions on diffrent modules in parellel. 
   * <p> This command runs the void function {@link #MoveHangDown} to run the motors down on each individual module.
   * <p> This command also runs {@link #HangIsAtPosition}, which checks when both hangers are down
   * <p> When both hangers, are down, it will run the {@link #HangStop} void function.
   * <p> If we were to run commands instead of void functions within the {@link Hanger#LowerHangAuto} command, it would not work.
   * <p> The command scheduler can only run 1 command at the same time within in a subsystem.
   * <p> That means if there are multiple commands being called at the same time, one of the commands would not run.
   * <p> Since we are calling the {@link Hanger#LowerHangAuto} command, all the commands within it would not be able to run if they were commands.
   * <p> But, since there are void functions, they are not required to be called by the Command scheduler, and can run smoothly.
  */


  public static int Explanation() {
    return 3;
  }
}
