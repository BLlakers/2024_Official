package frc.robot.Other;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.*;

public class SubsystemGetter extends SubsystemBase {

  private DriveTrain m_DriveTrain = new DriveTrain(Constants.CurrentRobotVersion());
  private Intake m_Intake = new Intake();
  private Hanger m_Hanger = new Hanger();
  private Limelight m_Limelight = new Limelight();
  private Shooter m_Shooter = new Shooter();
  private IntakeWheels m_IntakeWheels = new IntakeWheels();

  public IntakeWheels IntakeWheelsSub() {
    return m_IntakeWheels;
  }

  public DriveTrain DriveTrainSub() {
    return m_DriveTrain;
  }

  public Hanger HangSub() {
    return m_Hanger;
  }

  public Limelight LimelightSub() {
    return m_Limelight;
  }

  public Shooter ShooterSub() {
    return m_Shooter;
  }

  public Intake IntakeSub() {
    return m_Intake;
  }
}
