package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import frc.robot.subsystems.*;

public class HangCommand extends Command {
    private Hanger m_hanger;
    private BooleanSupplier m_ControllerButton;
    // private Supplier<Rotation3d> m_robotOrientationSupplier;
    // private PIDController m_hangingController = new PIDController(1, 0, 0);

    // private static final double GOAL_HEIGHT = 0.5; // meters
    // private static final double GOAL_ORIENTATION = 0.0; // radians

    // Calibrate THESE!!! TODO
    double speed = 0.5;
    double tolerance = 0.5;
    double ff = 0.1;

    public enum HangState {
        hangUp,
        hangDown,
    }

    public HangState CurrentHangState;

    // public HangCommand(Supplier<Rotation3d> robotOrientationSupplier){ //
    // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
    // m_robotOrientationSupplier = robotOrientationSupplier;
    // }

    public HangCommand(Hanger hang, BooleanSupplier button) {
        m_ControllerButton = button;
        m_hanger = hang;
        addRequirements(m_hanger);
    }

    @Override
    public void initialize() {
        m_hanger.ResetHangEnc();
    }

    @Override
    public void execute() {
        if (m_ControllerButton.getAsBoolean() == true) {
            CurrentHangState = HangState.hangUp;
            if (m_hanger.GetLeftPosition() >= 140) {
                m_hanger.LeftHangStop();
            }
            else {
                m_hanger.LeftHangUp();
            }
            if (m_hanger.GetRightPosition() >= 140) {
                m_hanger.RightHangStop();
            }
            else {
                m_hanger.RightHangUp(); 
            }
        } else {
            CurrentHangState = HangState.hangDown;
            if (m_hanger.RightHangIsDown() == true) {
                m_hanger.RightHangStop();
            } else {
                m_hanger.RightHangDown();
            }

            if (m_hanger.LeftHangIsDown() == true) {
                m_hanger.LeftHangStop();
            } else {
                m_hanger.LeftHangDown();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hanger.LeftHangStop();
        m_hanger.RightHangStop();
    }

}
