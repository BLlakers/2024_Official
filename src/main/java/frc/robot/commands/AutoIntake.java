package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        System.out.println(m_CurrentIntakeDrivingState);
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
            System.out.println(0);
            // if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosDownAngle) {
            if (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosDownAngle - 50) {
                System.out.println(1);
                m_Intake.LowerIntake();
            } else {
                System.out.println(2);
                m_Intake.StopIntake();
            }
            if (!m_IntakeWheels.NoteIsLoaded()) {
                System.out.println(3);
                m_IntakeWheels.RunIntakeWheels(); // inverted
            } else {
                System.out.println(4);
                m_IntakeWheels.StopIntakeWheels();
                m_CurrentIntakeDrivingState = DrivingState.DriveIntakeUp;
            }
        }
        else if (m_CurrentIntakeDrivingState == DrivingState.DriveIntakeUp) {
              System.out.println(5);
            if (m_Intake.GetIntakeMotorAngle().getDegrees() > Intake.PosUpAngle + 40) {
                System.out.println(6);
                m_Intake.RaiseIntake();
                
            } else {
                System.out.println(7);
                m_Intake.StopIntake();
                m_CurrentIntakeDrivingState = DrivingState.DriveIntakeDown;
                m_CommandIsFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("m_CurrentIntakeDrivingState");
        if (interrupted) {

            while (m_Intake.GetIntakeMotorAngle().getDegrees() < Intake.PosUpAngle) {
                m_Intake.RaiseIntake();
            }

            m_Intake.StopIntake();

        }
        m_Intake.StopIntake();
        m_IntakeWheels.StopIntakeWheels();
        

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
