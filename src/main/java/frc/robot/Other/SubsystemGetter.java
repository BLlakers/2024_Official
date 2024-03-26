package frc.robot.Other;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.*;
public class SubsystemGetter extends SubsystemBase{ 

    private DriveTrain m_DriveTrain = new DriveTrain(Constants.CurrentRobotVersion());
    private Intake m_Intake = new Intake();
    private Hanger m_Hanger = new Hanger();
    private Limelight m_Limelight = new Limelight();
    private Shooter m_Shooter = new Shooter();
    private IntakeWheels m_IntakeWheels = new IntakeWheels();
    
    public IntakeWheels GetIntakeWheels() {
        return m_IntakeWheels;
      }


    public DriveTrain GetDriveTrain() {
        return m_DriveTrain;
      }


    public Hanger GetHanger() {
        return m_Hanger;
      }


    public Limelight GetLimelight() {
        return m_Limelight;
      }

    public Shooter GetShooter() {
        return m_Shooter;
      }

    public Intake GetIntake() {
        return m_Intake;
      }
    public void SmartDashboardSetup(){
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(m_Shooter);
    SmartDashboard.putData(m_Hanger);
    SmartDashboard.putData(m_Intake);
    SmartDashboard.putData(m_IntakeWheels);
    SmartDashboard.putData(m_Limelight);
    SmartDashboard.putData(m_DriveTrain.getName() + "/Reset Pose 2D", m_DriveTrain.resetPose2d());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }
}
