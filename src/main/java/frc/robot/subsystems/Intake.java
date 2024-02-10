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

public class Intake extends SubsystemBase {


    public CANSparkMax intakeMtr = new CANSparkMax(Constants.intakeMtrChannel,
      com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    public Intake(){
    }

    @Override

    public void periodic() {
    }

    public Command RunIntake() {
        return run(
            () -> {
            intakeMtr.set(0.25);
            });
    }
    public Command StopIntake() {
        return run(
            () -> {
            intakeMtr.set(0);
            });
    }

}
