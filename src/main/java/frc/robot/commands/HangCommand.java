package frc.robot.commands;

// TODO FILE SHOULD BE A COMMAND, NOT A SUB
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class HangCommand extends Command {
    private Hanger m_HangerSub;
    public boolean ControllerAHeld;
    public Supplier<Rotation3d> m_robotOrientationSupplier;
    public PIDController m_hangingController = new PIDController(1, 0, 0);

    private static final double GOAL_HEIGHT = 0.5; // meters
    private static final double GOAL_ORIENTATION = 0.0; // radians
    public AHRS navx = new AHRS();

    //Calibrate THESE!!! TODO
    double speed = 0.5;
    double tolerance = 0.5; 
    double ff = 0.1;

    public enum HangState{
        hangUp,
        hangDown,
        hangLimit
    }
    public HangState CurrentHangState;


    // public HangCommand(Supplier<Rotation3d> robotOrientationSupplier){ // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
    //     m_robotOrientationSupplier = robotOrientationSupplier;
    // }

    public HangCommand(Hanger hang){
        m_HangerSub = hang;
        addRequirements(m_HangerSub);
    }

    @Override
    public void initialize() {
        m_HangerSub.ResetHangEnc();
    }

    @Override
    public void execute() {
        navx.getRotation3d();
        // armRotationMtr1.follow(armRotationMtr2);

        // double roll, pitch, yaw;
        // roll  = robotOrientation.getX(); // radians
        // pitch = robotOrientation.getY(); // radians 
        // yaw   = robotOrientation.getZ(); // radians

        // SmartDashboard.putNumber("Robot/Navx/Orientation/Roll",  Units.radiansToDegrees(roll));
        // SmartDashboard.putNumber("Robot/Navx/Orientation/Pitch", Units.radiansToDegrees(pitch));
        // SmartDashboard.putNumber("Robot/Navx/Orientation/Yaw",   Units.radiansToDegrees(yaw));

        if (ControllerAHeld = true){
            if (CurrentHangState == HangState.hangUp){
                if(m_HangerSub.hangerLeftMtrEnc.getPosition()  >= 140 && m_HangerSub.hangerRightMtrEnc.getPosition()  >= 140){
                    m_HangerSub.LeftHangStop();
                    m_HangerSub.RightHangStop();
                    } if(ControllerAHeld = false) {
                    CurrentHangState = HangState.hangDown;  
                    }
                }else if(m_HangerSub.hangerLeftMtrEnc.getPosition()  >= 140){
                    m_HangerSub.hangerLeftMtr.set(0);
                }else if(m_HangerSub.hangerRightMtrEnc.getPosition()  >= 140){
                    m_HangerSub.hangerRightMtr.set(0);
                }else {
                    m_HangerSub.hangerLeftMtr.set(0.9);
                    m_HangerSub.hangerRightMtr.set(0.9);
                }
        }
      /*   else if (CurrentHangState == HangState.hangDown){
            if(m_HangerSub.hangerLeftMtrEnc.getVelocity()  >= -3000 && m_HangerSub.hangerRightMtrEnc.getVelocity()  >= -3000){
                CurrentHangState = HangState.HangStop;
            }else if(m_HangerSub.hangerLeftMtrEnc.getVelocity()  >= -3000){
                m_HangerSub.hangerLeftMtr.set(0);
            }else if(m_HangerSub.hangerRightMtrEnc.getVelocity()  >= -3000){
                m_HangerSub.hangerRightMtr.set(0);
            }else{
                m_HangerSub.hangerLeftMtr.set(-0.9);
                m_HangerSub.hangerRightMtr.set(-0.9);
            }
*/ // Is this needed? }
        else if(CurrentHangState == HangState.hangDown) { 
            if(m_HangerSub.hangerLeftMtrEnc.getPosition()  <= 10 && m_HangerSub.hangerRightMtrEnc.getPosition()  <= 10){
                CurrentHangState = HangState.hangLimit;
            }else if(m_HangerSub.hangerLeftMtrEnc.getPosition()  <= 10){
                m_HangerSub.hangerLeftMtr.set(0);
            }else if(m_HangerSub.hangerRightMtrEnc.getPosition()  <= 10){
                m_HangerSub.hangerRightMtr.set(0);
            }else{
                m_HangerSub.hangerLeftMtr.set(-0.9);
                m_HangerSub.hangerRightMtr.set(-0.9);
            }
        }else if(CurrentHangState == HangState.hangLimit){
            if (m_HangerSub.LimitSwitchRightGet() == true){
                m_HangerSub.RightHangStop().schedule();
            } else {
                m_HangerSub.RightHangDown().schedule();
            }
            
            if(m_HangerSub.LimitSwitchLeftGet() == true){
                m_HangerSub.LeftHangStop().schedule();
            }else{
                m_HangerSub.LeftHangDown().schedule();
            }
        
        }else{
            m_HangerSub.hangerLeftMtr.set(0);
            m_HangerSub.hangerRightMtr.set(0);
        }
        

    }

    @Override
    public boolean isFinished(){
            return false;
        }

    @Override
    public void end(boolean interrupted) {
        m_HangerSub.hangerLeftMtr.set(0);
        m_HangerSub.hangerRightMtr.set(0);
    }

}


