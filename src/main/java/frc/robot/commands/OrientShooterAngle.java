package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Shooter;

public class OrientShooterAngle extends ProfiledPIDCommand {
  private Shooter m_Shooter;
  private boolean m_dynamicAngling;

  private static double s_kP = 0.5;
  private static double s_kI = 0.0;
  private static double s_kD = 0.0;

  public static Rotation2d s_DefaultAngle =
      Rotation2d.fromDegrees(55); // TODO: to be changed based on experiment

  private static final double s_maxShooterAnglingVelocity =
      Shooter.s_angleMotorSpeedPercentage; // motor percentage power
  private static final double s_maxShooterAnglingAcceleration =
      s_maxShooterAnglingVelocity / 2; // motor percentage power

  /**
   * Orient the shooter angle to a fixed goal
   *
   * @param shooter - the shooter subsystem
   * @param goal - Rotation2d object of the goal
   */
  public OrientShooterAngle(Shooter shooter, Rotation2d goal) {
    super(
        new ProfiledPIDController(
            s_kP,
            s_kI,
            s_kD,
            new TrapezoidProfile.Constraints(
                s_maxShooterAnglingVelocity, s_maxShooterAnglingAcceleration)),
        () -> {
          return shooter.GetShooterAngle().getRadians();
        },
        goal.getRadians(),
        (output, setpoint) -> shooter.SetShooterAngleSpeedPercentage(output),
        shooter);

    getController().enableContinuousInput(-Math.PI, Math.PI);
    m_Shooter = shooter;
    m_dynamicAngling = false;
  }

  /**
   * Orient the shooter angle adaptively with a dynamic goal supplier
   *
   * @param shooter - the shooter subystem
   * @param goalSupplier - the dynamic goal supplier for the angle to achieve (in radians)
   */
  public OrientShooterAngle(Shooter shooter, DoubleSupplier goalSupplier) {
    super(
        new ProfiledPIDController(
            s_kP,
            s_kI,
            s_kD,
            new TrapezoidProfile.Constraints(
                s_maxShooterAnglingVelocity, s_maxShooterAnglingAcceleration)),
        () -> {
          return shooter.GetShooterAngle().getRadians();
        },
        goalSupplier,
        (output, setpoint) -> shooter.SetShooterAngleSpeedPercentage(output),
        shooter);

    getController().enableContinuousInput(-Math.PI, Math.PI);
    m_Shooter = shooter;
    m_dynamicAngling = true;
  }

  /**
   * Provides a default command constructor to a preset desired default angle
   *
   * <p>This is intended as a backup for running this command with the pre-defined optimal shooter
   * angle (used for up close shooting)
   *
   * @param shooter - the Robot's "Shooter" subsystem
   * @return OrientShooterAngle command set for the default angle
   */
  public static OrientShooterAngle DefaultOrientShooterAngleCommand(Shooter shooter) {
    return new OrientShooterAngle(shooter, OrientShooterAngle.s_DefaultAngle);
  }

  @Override
  public void initialize() {
    super.initialize();
    getController().setTolerance(Units.degreesToRadians(0.5)); // set 0.5 degree tolerance for error
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_Shooter.AngleStop();
  }

  @Override
  public boolean isFinished() {
    if (m_dynamicAngling) // if you are angling the shooter dynamically, don't stop moving it
    return false;
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
    return 5;
  }
}
