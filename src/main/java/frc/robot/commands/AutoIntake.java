package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntake extends Command {
    Intake m_Intake;
    int TargetDeg;
    public static int Pos2 = 68; //  Down
    public static int Pos1 = 0; // starting
    public double CurrentIntakePose;
    public double IntakeTolerence = 3;
    public double StaticSetpointDeg = 30; // This needs to be measured TODO

    public AutoIntake(Intake IntakeSub) {

        m_Intake = IntakeSub;

        addRequirements(m_Intake);

    }

public void execute(){
    CurrentIntakePose = m_Intake.GetIntakeMotorAngle().getDegrees();
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







}

}
