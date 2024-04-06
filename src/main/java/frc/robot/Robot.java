package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// some imports no longer needed but leaving them here untill final version

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  String codeVersion = "0.0";
  private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);

  // commit
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.GetDriveTrainSub().ZeroGyro().schedule();
    CameraServer.startAutomaticCapture();

    SmartDashboard.putString("Code Version", codeVersion);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData(PDH);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.GetIntakeSub().resetIntakeAngle();
    m_robotContainer.GetDriveTrainSub().m_FieldRelativeEnable = false;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.GetDriveTrainSub().m_FieldRelativeEnable = true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public interface RobotExplanation {}
}
