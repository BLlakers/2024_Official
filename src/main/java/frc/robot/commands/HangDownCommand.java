package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class HangDownCommand extends Command {
private Hanger m_HangerSub; 
public int HangState;
    public HangDownCommand(Hanger HangerSub){
    
    m_HangerSub = HangerSub;
    addRequirements(m_HangerSub);
    }
    public void execute(){
    if(m_HangerSub.hangerLeftMtrEnc.getVelocity()  >= -3000 && m_HangerSub.hangerRightMtrEnc.getVelocity()  >= -3000){
        HangState = 2;
    }else if(m_HangerSub.hangerLeftMtrEnc.getVelocity()  >= -3000){
        m_HangerSub.hangerLeftMtr.set(0);
    }else if(m_HangerSub.hangerRightMtrEnc.getVelocity()  >= -3000){
        m_HangerSub.hangerRightMtr.set(0);
    }else{
        m_HangerSub.hangerLeftMtr.set(-0.9);
        m_HangerSub.hangerRightMtr.set(-0.9);
    }
     if (HangState == 2) { 
    if(m_HangerSub.hangerLeftMtrEnc.getPosition()  <= 10 && m_HangerSub.hangerRightMtrEnc.getPosition()  <= 10){
        HangState = 3;
    }else if(m_HangerSub.hangerLeftMtrEnc.getPosition()  <= 10){
        m_HangerSub.hangerLeftMtr.set(0);
    }else if(m_HangerSub.hangerRightMtrEnc.getPosition()  <= 10){
        m_HangerSub.hangerRightMtr.set(0);
    }else{
        m_HangerSub.hangerLeftMtr.set(-0.9);
        m_HangerSub.hangerRightMtr.set(-0.9);
    }
}else if(HangState == 3 ){
    m_HangerSub.hangerLeftMtr.set(0);
    m_HangerSub.hangerRightMtr.set(0);
}else{
    m_HangerSub.hangerLeftMtr.set(0);
    m_HangerSub.hangerRightMtr.set(0);
}
isFinished();
}
}