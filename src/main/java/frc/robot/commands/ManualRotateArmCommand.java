package frc.robot.commands;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
public class ManualRotateArmCommand extends Command {
  DoubleSupplier m_leftY;
  Arm m_Arm;

  public ManualRotateArmCommand(DoubleSupplier _leftY, Arm _Arm) {
    m_leftY = _leftY;
    m_Arm = _Arm;
    addRequirements(m_Arm);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // double m_sensorPosition = -m_Arm.armRotationMtr.getSelectedSensorPosition(); // Variable to hold the sensor position
    // m_Arm.ArmDegrees = (m_sensorPosition * 360) / (120 * 4096); // 120:1 gearbox

    double controllerValue = m_leftY.getAsDouble();


      // setting the Deadzone value of the controller. It tells
      // the controller not to move at .1> (a small amount on the controller) and to
      // move everywhere else.
      if (Math.abs(controllerValue) < Constants.deadzone) {
        controllerValue = 0;
      } else {
        controllerValue = controllerValue;
      }

      controllerValue = controllerValue * 0.5;

    // Limit switch is inverted logic
    
      // m_Arm.armRotationMtr.setSelectedSensorPosition(5*120 *4096 /360);
      if (controllerValue >= 0) {
        // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 0 * controllerValue);
       } else {
        // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, controllerValue);
      



      // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, .5 * controllerValue);
      /*
       * The following code reads the motor encoder sensor and sets the percentoutput
       * to 0
       * This proof of concept needs further logic added to continue moving below the
       * threshold, in the opposite direction
       */

      // If we are at the limit
      if (m_Arm.ArmDegrees >= 90) {
        // Then don't go further
        if (controllerValue <= 0) {
          // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 0 * controllerValue);
        } else {
          // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 1 * controllerValue);
        }
      } else {
        // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 1 * controllerValue);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 0);
  }

  public boolean isFinished() {
    return false;
  }

}
