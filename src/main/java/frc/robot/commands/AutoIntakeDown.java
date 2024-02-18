package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeDown extends Command {
    Intake m_Intake;
    int TargetDeg;
    public static int Pos2 = 68; //  Down
    public static int Pos1 = 0; // starting
    public double CurrentIntakePose;
    public double IntakeTolerence = 3;
    public double StaticSetpointDeg = 30; // This needs to be measured TODO

    @Override
public void initialize(){
}
    public AutoIntakeDown(Intake IntakeSub) {

        m_Intake = IntakeSub;

        addRequirements(m_Intake);

    }
@Override
public void execute(){
    if (m_Intake.GetIntakeMotorAngle().getDegrees() < Pos2){
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
    }

}
@Override
public void end(boolean inerrupted){
    m_Intake.intakeAngleMtr.set(0);
    m_Intake.intakeWheelMtrL.set(0); // inverted
    m_Intake.intakeWheelMtrR.set(0);
}
@Override
public boolean isFinished(){
    if (m_Intake.IR < 40){
    return false;
   // }else{
    //    return true;
   // }
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




return false;
}
}


