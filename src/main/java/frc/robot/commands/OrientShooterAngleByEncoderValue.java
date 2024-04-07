package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Shooter;

public class OrientShooterAngleByEncoderValue extends ProfiledPIDCommand {
  public static final double ANGLE_TOP = 0;
  public static final double ANGLE_2 = 0; // todo: find a middle position

  private static double s_kP = 0.5;
  private static double s_kI = 0.0;
  private static double s_kD = 0.0;

  private static final double s_tolerance = 20;

  private static final double s_maxShooterAnglingVelocity =
      Shooter.s_angleMotorSpeedPercentage; // motor percentage power
  private static final double s_maxShooterAnglingAcceleration =
      s_maxShooterAnglingVelocity / 2; // motor percentage power

  private Shooter m_Shooter;

  /**
   * Orient the shooter angle to a fixed goal
   *
   * @param shooter - the shooter subsystem
   * @param goal - Rotation2d object of the goal
   */
  public OrientShooterAngleByEncoderValue(Shooter shooter, double goal) {
    super(
        new ProfiledPIDController(
            s_kP,
            s_kI,
            s_kD,
            new TrapezoidProfile.Constraints(
                s_maxShooterAnglingVelocity, s_maxShooterAnglingAcceleration)),
        shooter::GetHeightAlongLeadScrew,
        goal,
        (output, setpoint) -> shooter.SetShooterAngleSpeedPercentage(output),
        shooter);

    getController().enableContinuousInput(-Math.PI, Math.PI);
    m_Shooter = shooter;
  }

  @Override
  public void initialize() {
    super.initialize();
    getController().setTolerance(s_tolerance); // set 0.5 degree tolerance for error
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_Shooter.AngleStop();
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
/** <b> DETAILED EXPLANATION </b>
   * 
   * 
   * 
   * 
   * 
   */
  public static int Explanation() {
    return 6;
  }
}
