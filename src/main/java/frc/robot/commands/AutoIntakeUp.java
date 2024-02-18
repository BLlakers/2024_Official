package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeUp extends Command {
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
    public AutoIntakeUp(Intake IntakeSub) {

        m_Intake = IntakeSub;

        addRequirements(m_Intake);

    }
@Override
public void execute(){
    if (m_Intake.GetIntakeMotorAngle().getDegrees() > Pos1){
        m_Intake.intakeAngleMtr.set(-.35);
    } else {
        m_Intake.intakeAngleMtr.set(0);
    }





}

}
