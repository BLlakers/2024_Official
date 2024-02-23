package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * My idea on how AutoIntake should be done. Uses state machine + ben's
 * Pre-Written code.
 */

public class AutoIntake extends Command {
    private Intake m_Intake;
    private boolean m_IsAmp = false;

    // determines where we want to drive the intake angle to
    public enum DrivingState {
        DriveIntakeUp,
        DriveIntakeDown,
        DriveIntakeAmp,
    };

    private DrivingState m_CurrentIntakeDrivingState;

    private static final double s_IntakePositioningTolerence = 5;
    private boolean m_CommandIsFinished = false;

    @Override
    public void initialize() {
        m_IsAmp = false;
    }

    public AutoIntake(Intake IntakeSub) {
        m_Intake = IntakeSub;

        if (m_Intake.GetIntakeState() == Intake.State.PositionUp)
            m_CurrentIntakeDrivingState = DrivingState.DriveIntakeDown;

        else
            m_CurrentIntakeDrivingState = DrivingState.DriveIntakeUp;

        addRequirements(m_Intake);
    }

    public AutoIntake(Intake IntakeSub, DrivingState forcedDrivingState)
    {
        m_Intake = IntakeSub;
        m_CurrentIntakeDrivingState = forcedDrivingState;

        addRequirements(m_Intake);
    }

    public void SetIsAmpFlag(boolean isAmp) {
        this.m_IsAmp = isAmp;  
    }

    @Override
    public void execute() {
      /*   if (m_IsAmp == true) {
            m_CurrentIntakeDrivingState = DrivingState.DriveIntakeAmp;
            if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosAmpAngle + s_IntakePositioningTolerence) {
                m_Intake.RaiseIntake();
            } else if (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosAmpAngle - s_IntakePositioningTolerence) {
                m_Intake.LowerIntake();
            } else if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosAmpAngle + s_IntakePositioningTolerence
                    && m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosAmpAngle - s_IntakePositioningTolerence) {
                m_Intake.StopIntake();
                m_CommandIsFinished = true;
            }

        }






        else */
        if (m_CurrentIntakeDrivingState == DrivingState.DriveIntakeDown) {
            if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosDownAngle) {
                m_Intake.LowerIntake().schedule();
            } else {
                m_Intake.StopIntake().schedule();
            }
            if (!m_Intake.NoteIsLoaded()) {
                m_Intake.RunIntakeWheels().schedule(); // inverted
            } else {
                m_Intake.StopIntakeWheels().schedule();
                m_CurrentIntakeDrivingState = DrivingState.DriveIntakeUp;
            }
        }
        if (m_CurrentIntakeDrivingState == DrivingState.DriveIntakeUp) {
            if (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosUpAngle) {
                m_Intake.RaiseIntake().schedule();
            } else {
                m_Intake.StopIntake().schedule();
                m_CommandIsFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
        {
            
            while (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosUpAngle) {
                m_Intake.RaiseIntake().schedule();
            } 
            
            m_Intake.StopIntake().schedule();
            
        }
        m_Intake.StopIntake().schedule();
        m_Intake.StopIntakeWheels().schedule();
    }

    @Override
    public boolean isFinished() {
        return m_CommandIsFinished;
    }
}
/*
 * CurrentIntakePose = m_Intake.GetIntakeMotorAngle().getDegrees();
 * if (m_Intake.IntakePos == 1){
 * TargetDeg = Pos1;
 * if (CurrentIntakePose > TargetDeg + IntakeTolerence){
 * if (CurrentIntakePose > StaticSetpointDeg){
 * m_Intake.intakeAngleMtr.set(-.25);
 * } else {
 * m_Intake.intakeAngleMtr.set(-.45);
 * }
 * } else{
 * m_Intake.intakeAngleMtr.set(0);
 * m_Intake.intakeAngleMtrEnc.setPosition(0);
 * }
 * }
 * 
 * if (m_Intake.IntakePos == 2){
 * TargetDeg = Pos2;
 * if (CurrentIntakePose < TargetDeg - IntakeTolerence){
 * m_Intake.intakeAngleMtr.set(.17);
 * }
 * else {
 * m_Intake.intakeAngleMtr.set(0);
 * }
 * }
 * 
 */
