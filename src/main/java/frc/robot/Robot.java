package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveModule;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

//some imports no longer needed but leaving them here untill final version

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Arm m_Arm;
  String codeVersion = "0.0";
  
  //color sensor, it communicates over the only i2c port onboard
  private final I2C.Port i2c =I2C.Port.kOnboard;
  private final ColorSensorV3 m_ColorSensor = new ColorSensorV3(i2c);
  // commit
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // camera stuff edited at comp
    // UsbCamera camera = new UsbCamera("cam0", 0);
    // camera.setFPS(15);
    // camera.setResolution(480, 320);
    CameraServer.startAutomaticCapture();

    SmartDashboard.putString("Code Version", codeVersion);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //dose as it sounds, prints proximity
    int proximity = m_ColorSensor.getProximity();
    System.out.println(proximity);
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    //System.out.println(m_robotContainer.m_DriveTrainPID.m_frontRight.m_turningEncoder.getAbsolutePosition());
    //System.out.println(m_robotContainer.m_DriveTrainPID.m_backRight.m_driveEncoder.getPosition());
  }

  @Override
  public void autonomousInit() {
    // m_robotContainer.m_Arm.ArmPosition = 1;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.m_DriveTrain.startYaw =
    // m_robotContainer.m_DriveTrain.getGyroYaw();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // m_robotContainer.m_Arm.ArmPosition = 1;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
    // m_robotContainer.m_DriveTrain.startYaw =
    // m_robotContainer.m_DriveTrain.getGyroYaw();

  }

  @Override
  public void teleopPeriodic() {
    //System.out.println(m_robotContainer.m_DriveTrainPID.m_backRight.m_turningEncoder.getAbsolutePosition());
    //System.out.println(m_robotContainer.m_DriveTrainPID.m_frontRight.m_turningEncoder.getAbsolutePosition());
   
    // WP - Was not compiling as of 3/4, to be addressed
    // cameraTest();
    // SmartDashboard.putNumber("Start Yaw",
    // m_robotContainer.m_DriveTrain.startYaw);

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
