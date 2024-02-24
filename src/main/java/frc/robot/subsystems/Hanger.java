package frc.robot.subsystems;
// TODO FILE SHOULD BE A COMMAND, NOT A SUB
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ArmFeedforward;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Hanger extends SubsystemBase{
    public DigitalInput hangRightLimitSwitch = new DigitalInput(9);
    public DigitalInput hangLeftLimitSwitch = new DigitalInput(8);
    public CANSparkMax hangerLeftMtr = new CANSparkMax(Constants.hangerLeftMtrC,
        com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax hangerRightMtr = new CANSparkMax(Constants.hangerRightMtrC,
        com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    public RelativeEncoder hangerLeftMtrEnc = hangerLeftMtr.getEncoder();
    public RelativeEncoder hangerRightMtrEnc = hangerRightMtr.getEncoder(); 

    //public Supplier<Rotation3d> m_robotOrientationSupplier;
    //public PIDController m_hangingController = new PIDController(1, 0, 0);

    //private static final double GOAL_HEIGHT = 0.5; // meters
    //private static final double GOAL_ORIENTATION = 0.0; // radians
    //public AHRS navx = new AHRS();


    public Hanger (){ // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
        ResetHangEnc();
    }

    @Override

    public void periodic() {
       // navx.getRotation3d();
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Hanger/Left Hang Pos", hangerLeftMtrEnc.getPosition());
        SmartDashboard.putNumber("Hanger/Right Hang Pos", hangerRightMtrEnc.getPosition());

        SmartDashboard.putNumber("Hanger/Left Current",hangerLeftMtr.getOutputCurrent());
        SmartDashboard.putNumber("Hanger/Right Current",hangerRightMtr.getOutputCurrent());
        SmartDashboard.putNumber("Hanger/Left Hang Velo",hangerLeftMtrEnc.getVelocity());
        SmartDashboard.putNumber("Hanger/Right Hang Velo",   hangerRightMtrEnc.getVelocity());
        
    }

    public void ResetHangEnc(){
        hangerLeftMtrEnc.setPosition(0.0);
        hangerRightMtrEnc.setPosition(0.0);
    }
    

    public Command ResetHangCmd(){
        return runOnce(
            () -> {
                    ResetHangEnc();
            });
    }

    public Command LeftHangUp() {
        return run(
            () -> {
            hangerLeftMtr.set(0.75);
            });
      }

    public Command LeftHangDown() {
        return run(
            () -> {
            hangerLeftMtr.set(-0.75);
            });
      }

    public Command LeftHangStop() {
        return run(
            () -> {
            hangerLeftMtr.set(0);
            });
      }
    public Command RightHangUp() {
        return run(
            () -> {
            hangerRightMtr.set(0.75);
            });
      }

    public Command RightHangDown() {
        return run(
            () -> {
            hangerRightMtr.set(-0.75);
            });
      }

    public Command RightHangStop() {
        return run(
            () -> {
            hangerRightMtr.set(0);
            });
      }
      public boolean LimitSwitchLeftGet(){
       return hangLeftLimitSwitch.get();
      }
         public boolean LimitSwitchRightGet(){
       return hangRightLimitSwitch.get();
      }

    //   public double getHangVelocityEQ(double speed){
    //     if (hangerLeftMtrEnc.getVelocity() + Tolerance > hangerRightMtrEnc.getVelocity()){
    //     return hangerLeftMtrEnc.getVelocity() + Tolerance;
    //     hangerLeftMtr.set(speed-ff);
    //     hangerRightMtr.set(speed);
    //     }
    //     if (hangerLeftMtrEnc.getVelocity() < hangerRightMtrEnc.getVelocity() - Tolerance){
    //     return hangerRightMtrEnc.getVelocity() - Tolerance;
    //     hangerLeftMtr.set(speed);
    //     hangerRightMtr.set(speed-ff);
    //     }
    //   }


}
