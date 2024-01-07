package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AutoRotateArmCommand extends CommandBase {
  Arm m_Arm;
  Double targetDegrees;

  public AutoRotateArmCommand(Arm _Arm) {
    m_Arm = _Arm;
    addRequirements(m_Arm);

  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // double m_sensorPosition = m_Arm.armRotationMtr.getSelectedSensorPosition(); // Variable to hold the sensor position
    // m_Arm.ArmDegrees = (m_sensorPosition * 360 * 0.72) / (120 * 2048); // 120:1 gearbox
    

    // Determine the target position
    if (m_Arm.ArmPosition == 1) { // The target position for 0 = Lower, 1 = pickup, 2 = Drop
      //targetDegrees = Constants.Positions[0];
      //m_Arm.armRotationMtr.set(ControlMode.PercentOutput, -1);
      m_Arm.armRotationMtr2.set(-1);
      //m_Arm.armRotationMtr1.set(-1);
      System.out.println("JGANG$:");
      //System.out.println(m_Arm.armRotationMtr2.get());
    } else if (m_Arm.ArmPosition == 2) {
      //targetDegrees = Constants.Positions[1];
      //m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 0);
      m_Arm.armRotationMtr2.set(0);
      //m_Arm.armRotationMtr1.set(0);
      System.out.println("KKKKKKKKKKKKKKKKKKK");
      //System.out.println(m_Arm.armRotationMtr2.get());
    } else if (m_Arm.ArmPosition == 3) { 
      m_Arm.armRotationMtr2.set(1);
      //m_Arm.armRotationMtr1.set(1);
      System.out.println("JAJSJSJDJASJD");
      //System.out.println(m_Arm.armRotationMtr2.get());
      //m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 1);
    }
   // SmartDashboard.putNumber("Target", targetDegrees);
    SmartDashboard.putNumber("Arm Degrees", m_Arm.ArmDegrees);
    // Use bang bang control to reach the target
   

    // Apply a feedforward constant to hold the arm in position
    
  

    //if (!m_Arm.ArmLimitSwitch.get()) {
      // Logic if the limit switch is pressed
      // Set the encoder to the lower position, update the position we are at
      // accordingly

  }
  @Override
  public void end(boolean interrupted) {
    //m_Arm.armRotationMtr.set(ControlMode.PercentOutput, 0);
    m_Arm.armRotationMtr1.set(0);
    m_Arm.armRotationMtr2.set(0);
    System.out.println("??????");
  }

  public boolean isFinished() {
    return false;
  }

}
