package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class HangUpCommand extends Command {
private Hanger m_HangerSub;
public HangUpCommand(Hanger HangerSub){ // will eventually also be a PoseSupplier for the limelight tag
m_HangerSub = HangerSub;
addRequirements(m_HangerSub);
}
@Override
public void initialize() {
    m_HangerSub.ResetHangEnc();
}

    public void execute(){
   /*  if (Hanger.HangState == 0){
        if(m_HangerSub.hangerLeftMtrEnc.getPosition()  >= 140 && m_HangerSub.hangerRightMtrEnc.getPosition()  >= 140){
        Hanger.HangState = 1; 
        Hanger.HangType = "HangDown";
        isFinished();
        }else if(m_HangerSub.hangerLeftMtrEnc.getPosition()  >= 140){
            m_HangerSub.hangerLeftMtr.set(0);
        }else if(m_HangerSub.hangerRightMtrEnc.getPosition()  >= 140){
            m_HangerSub.hangerRightMtr.set(0);
        }else{
            m_HangerSub.hangerLeftMtr.set(0.9);
            m_HangerSub.hangerRightMtr.set(0.9);
        }
    }*/
    }
}
