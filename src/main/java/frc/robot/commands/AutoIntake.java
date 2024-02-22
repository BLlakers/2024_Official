package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;



/**
 * My idea on how AutoIntake should be done. Uses state machine + ben's Pre-Written code. 
 */

public class AutoIntake extends Command {
    Intake m_Intake;
    public static boolean IsAmp = false;
    int TargetDeg = 1;
    public static int PosDown = 68; //  Down
    public static int PosUp = 0; // starting
    public double CurrentIntakePose;
    public double IntakeTolerence = 5;
    public double AmpDeg = 30; // This needs to be measured TODO

    @Override
public void initialize(){
}
    public AutoIntake(Intake IntakeSub) {
        m_Intake = IntakeSub;

        addRequirements(m_Intake);

    }
@Override
public void execute(){
    if (IsAmp == true){
        TargetDeg = 0;
        if(m_Intake.GetIntakeMotorAngle().getDegrees() < AmpDeg + IntakeTolerence){
        m_Intake.intakeAngleMtr.set(.17);
        }
        else if (m_Intake.GetIntakeMotorAngle().getDegrees() > AmpDeg - IntakeTolerence){
               m_Intake.intakeAngleMtr.set(-.35);
        }
        else if (m_Intake.GetIntakeMotorAngle().getDegrees() < AmpDeg + IntakeTolerence && m_Intake.GetIntakeMotorAngle().getDegrees() > AmpDeg - IntakeTolerence){
             m_Intake.intakeAngleMtr.set(0);
        }

    } if (IsAmp == false) {
        TargetDeg = 1;
    }
    if (TargetDeg == 1){
        if (m_Intake.GetIntakeMotorAngle().getDegrees() < PosDown){
            m_Intake.intakeAngleMtr.set(.17);
        } else {
            m_Intake.intakeAngleMtr.set(0);
        } 
        if (m_Intake.IR < 40){
            m_Intake.intakeWheelMtrL.set(-.5); // inverted
            m_Intake.intakeWheelMtrR.set(-.5);
        } else {
            m_Intake.intakeWheelMtrL.set(0); // inverted
            m_Intake.intakeWheelMtrR.set(0);
            TargetDeg = 2;
        }   
    }
    if (TargetDeg ==2){
        if (m_Intake.GetIntakeMotorAngle().getDegrees() > PosUp){
            m_Intake.intakeAngleMtr.set(-.35);
        } else {
            m_Intake.intakeAngleMtr.set(0);
            TargetDeg = 1;
        }
    } 
    isFinished(); 
}
@Override
public void end(boolean inerrupted){
    m_Intake.intakeAngleMtr.set(0);
    m_Intake.intakeWheelMtrL.set(0); // inverted
    m_Intake.intakeWheelMtrR.set(0);
}
@Override
public boolean isFinished(){
    if (IsAmp == true){
        return true;
    }
    if( m_Intake.GetIntakeMotorAngle().getDegrees() > PosUp){
        return false;
    }else {
        return true;
    }
 }
}
  /*  CurrentIntakePose = m_Intake.GetIntakeMotorAngle().getDegrees();
    if (m_Intake.IntakePos == 1){
        TargetDeg = Pos1;
        if (CurrentIntakePose > TargetDeg + IntakeTolerence){
            if (CurrentIntakePose > StaticSetpointDeg){
                m_Intake.intakeAngleMtr.set(-.25);
            } else {
                m_Intake.intakeAngleMtr.set(-.45);
            }
    } else{
        m_Intake.intakeAngleMtr.set(0);
        m_Intake.intakeAngleMtrEnc.setPosition(0);
    }
}
    
    if (m_Intake.IntakePos == 2){
        TargetDeg = Pos2;
        if (CurrentIntakePose < TargetDeg - IntakeTolerence){
            m_Intake.intakeAngleMtr.set(.17);
        }
        else {
            m_Intake.intakeAngleMtr.set(0);
        }
    }

*/


   
   

   

 