package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

//import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {

  // product-specific voltage->pressure conversion, see product manual
  // in this case, 250(V/5)-25
  // the scale parameter in the AnalogPotentiometer constructor is scaled from 1
  // instead of 5,
  // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25
  // double scale = 250;
  // double offset = -25;
  // AnalogPotentiometer pressureTransducer = new AnalogPotentiometer(/* the
  // AnalogIn port*/ 2, scale, offset);


 
  // public TalonFX armRotationMtr = new TalonFX(Constants.armMotorChannel);

 // public int ArmPosition = 2;
  //public TalonFX armRotationMtr = new TalonFX(Constants.armMotorChannel2);
  //public TalonFX armRotationMtr2 = new TalonFX(Constants.armMotor2);
  public int ArmPosition = 1;
  public double ArmDegrees = 0;
  // scaled values in psi units
  // double psi = pressureTransducer.get();

  // boolean pressureSwitch = phCompressor.getPressureSwitchValue();
  public Arm() {
    // armRotationMtr1.setInverted(true);
    // armRotationMtr2.setInverted(true);
    // armRotationMtr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // armRotationMtr.setSelectedSensorPosition(85*120 *2048/360);
    // RelativeEncoder encoder = armRotationMtr.getEncoder();
    // double encoderPos = encoder.getPosition();

    // armRotationMtr.setInverted(true);
    // armRotationMtr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // armRotationMtr.setSelectedSensorPosition(85*120 *2048);
    // System.out.println(armRotationMtr.getSelectedSensorPosition());
    // RelativeEncoder encoder = armRotationMtr.getEncoder();
    // double encoderPos = encoder.getPosition();

  }

  @Override

  public void periodic() {
    // armRotationMtr1.follow(armRotationMtr2);

    SmartDashboard.putNumber("Arm Position", ArmPosition);
    SmartDashboard.putNumber("Arm Degrees", ArmDegrees);
    String jared = "Jared";

  }

  // This is is an inline command construction. Commands which are so quick and
  // easy as to only run a line one
  // (they have no start, periodic, end behavior) can be defined this way. Inline
  // commands are defined in the subsystem
  // instead of in a separate .java file in the commands folder.
  public Command RaiseArm() {
    return runOnce(
        () -> {
          // one-time action goes here
          ArmPosition = ArmPosition + 1;
          if (ArmPosition == 4) {
            ArmPosition = 3;
          }
        });
  }

  public Command LowerArm() {
    return runOnce(
        () -> {
          // one-time action goes here
          ArmPosition = ArmPosition - 1;
          if (ArmPosition == 0) {
            ArmPosition = 1;
          }
        });
  }

}
