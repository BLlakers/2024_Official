package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase  {
    
    public CANSparkMax intakeAngleMtr = new CANSparkMax(Constants.intakeAngleMtrC,
      com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax intakeWheelMtrL = new CANSparkMax(Constants.intakeWheelMtrL,
      com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax intakeWheelMtrR = new CANSparkMax(Constants.intakeWheelMtrR,
      com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public RelativeEncoder intakeAngleMtrEnc = intakeAngleMtr.getEncoder();
    public RelativeEncoder intakeWheelMtr1Enc = intakeWheelMtrL.getEncoder(); 
    public RelativeEncoder intakeWheelMtr2Enc =  intakeWheelMtrR.getEncoder();
    public int IntakePos = 1; 

    public static double GEAR_RATIO = 100.0;
    public static double TARGET_ANGLE = 68; // TODO: TARGET ANGLE IN DEGREES OF THE MOTOR
    //I set this at 410 to account for gravity orginal value was 445 -Ben
    public Intake(){
        // intakeWheelMtr1.follow(intakeWheelMtr2);
        intakeAngleMtrEnc.setPosition(0);
        intakeAngleMtrEnc.setPositionConversionFactor(2 * Math.PI / GEAR_RATIO);

    }

    @Override

    public void periodic() {
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Intake Position", GetIntakeMotorAngle().getDegrees());
    }

    public Command LowerIntake() {
        return run(
            () -> {
            intakeAngleMtr.set(.17);  
            });
      }

    public Command RaiseIntake() {
        return runOnce(
            () -> {
            intakeAngleMtr.set(-.45);
            });
      }

public Command StopIntake(){
    return runOnce(
     () -> {
        intakeAngleMtr.set(0);
    });
}
    public Command RunIntakeWheels() {
        return runOnce(
            () -> {
        intakeWheelMtrL.set(-.5); // inverted
        intakeWheelMtrR.set(-.5);
            });
      }

    public Command StopIntakeWheels() {
        return runOnce(
            () -> {
        intakeWheelMtrL.set(0);
        intakeWheelMtrR.set(0);
            });
      }
      public Command ReverseIntakeWheels() {
        return runOnce(
            () -> {
        intakeWheelMtrL.set(1);
        intakeWheelMtrR.set(-1);
            });
      }
    public Command AutoLowerIntake(){
        return run(
            () -> {
            if (GetIntakeMotorAngle().getDegrees() < TARGET_ANGLE){
                intakeAngleMtr.set(.17);
            }
            });
    }

    public Rotation2d GetIntakeMotorAngle()
    {
        return Rotation2d.fromRadians(intakeAngleMtrEnc.getPosition());
    }
}
