package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntake extends Command {
    private Intake m_Intake;
    private IntakeWheels m_IntakeWheels;
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
        // m_Intake.resetIntakeAngle();
    }

    public AutoIntake(Intake IntakeSub, IntakeWheels IntakeWheelsSub) {
        m_Intake = IntakeSub;
        m_IntakeWheels = IntakeWheelsSub;

        if (m_Intake.GetIntakeState() == Intake.State.PositionUp)
            m_CurrentIntakeDrivingState = DrivingState.DriveIntakeDown;

        else
            m_CurrentIntakeDrivingState = DrivingState.DriveIntakeUp;

        addRequirements(m_Intake, m_IntakeWheels);
    }

    public AutoIntake(Intake IntakeSub, IntakeWheels IntakeWheelsSub, DrivingState forcedDrivingState) {
        m_Intake = IntakeSub;
        m_IntakeWheels = IntakeWheelsSub;

        m_CurrentIntakeDrivingState = forcedDrivingState;

        addRequirements(m_Intake, m_IntakeWheels);
    }

    public void SetIsAmpFlag(boolean isAmp) {
        this.m_IsAmp = isAmp;
    }

    @Override
    public void execute() {
        /*
         * if (m_IsAmp == true) {
         * m_CurrentIntakeDrivingState = DrivingState.DriveIntakeAmp;
         * if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosAmpAngle +
         * s_IntakePositioningTolerence) {
         * m_Intake.RaiseIntake();
         * } else if (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosAmpAngle -
         * s_IntakePositioningTolerence) {
         * m_Intake.LowerIntake();
         * } else if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosAmpAngle +
         * s_IntakePositioningTolerence
         * && m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosAmpAngle -
         * s_IntakePositioningTolerence) {
         * m_Intake.StopIntake();
         * m_CommandIsFinished = true;
         * }
         * 
         * }
         * 
         * 
         * 
         * 
         * 
         * 
         * else
         */
        if (m_CurrentIntakeDrivingState == DrivingState.DriveIntakeDown) {
            // if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosDownAngle) {
            if (m_Intake.intakeAngleMtrEnc.getPosition() >= IntakeWheels.PositionDown) {
                m_Intake.LowerIntake().schedule();
            } else {
                m_Intake.StopIntake().schedule();
            }
            if (!m_IntakeWheels.NoteIsLoaded()) {
                m_IntakeWheels.RunIntakeWheels().schedule(); // inverted
            } else {
                m_IntakeWheels.StopIntakeWheels().schedule();
                m_CurrentIntakeDrivingState = DrivingState.DriveIntakeUp;
            }
        }
        if (m_CurrentIntakeDrivingState == DrivingState.DriveIntakeUp) {
            // if (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosUpAngle) {
            if (m_Intake.intakeAngleMtrEnc.getPosition() <= IntakeWheels.PositionUp) {
                m_Intake.RaiseIntake().schedule();
            } else {
                m_Intake.StopIntake().schedule();
                m_CommandIsFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {

            while (m_Intake.GetIntakeMotorAngle().getDegrees() > IntakeWheels.PosUpAngle) {
                m_Intake.RaiseIntake().schedule();
            }

            m_Intake.StopIntake().schedule();

        }
        m_Intake.StopIntake().schedule();
        m_IntakeWheels.StopIntakeWheels().schedule();
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
