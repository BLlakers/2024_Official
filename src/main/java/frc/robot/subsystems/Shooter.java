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

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
    
    public CANSparkMax shooterMtrLeft = new CANSparkMax(Constants.shooterMtrLeftC,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  public CANSparkMax shooterMtrRight = new CANSparkMax(Constants.shooterMtrRightC,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  public CANSparkMax ShooterAngleMtr = new CANSparkMax(Constants.shooterAngleMtrC,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public RelativeEncoder shooterMtrLeftEnc = shooterMtrLeft.getEncoder();
    public RelativeEncoder shooterMtrRightEnc = shooterMtrRight.getEncoder(); 
    public RelativeEncoder AngleMtrEnc =  ShooterAngleMtr.getEncoder();

    public Shooter (){
        //shooterMtrLeft.follow(shooterMtrRight, true);
    }


    @Override

    public void periodic() {
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMtrLeftEnc.getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Angle", AngleMtrEnc.getPosition());
    }


    public Command RunShooter() {
        return runOnce(
            () -> {
            shooterMtrLeft.set(.4);
            shooterMtrRight.set(.4);
            });
      }
    public Command StopShooter() {
        return run(
        () -> {
            shooterMtrLeft.set(0);
            shooterMtrRight.set(0);
        });
    }


     public Command AngleDownShooter() {
        return run(
        () -> {
            ShooterAngleMtr.set(.95);
        });
    }

     public Command AngleUpShooter() {
        return run(
        () -> {
            ShooterAngleMtr.set(-.95);
        });
    }

         public Command AngleStop() {
        return run(
        () -> {
            ShooterAngleMtr.set(0);
        });
    }
    
}
