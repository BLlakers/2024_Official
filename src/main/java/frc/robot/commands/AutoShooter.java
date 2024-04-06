package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {
  private final Supplier<AprilTag> m_aprilTagProvider;
  private final Shooter m_shooter;
  private final BooleanSupplier m_shooterLoaded;

  private final OrientShooterAngle m_orientShooterAngleCommand;
  private final AprilAlignToSpeakerRadiallyCommand m_aprilAlignCommand;

  private static final Transform3d APRILTAG_TO_SHOOTINGTARGET =
      new Transform3d(
          Units.inchesToMeters(6), // x: back of target (needs to be tuned)
          0, // y
          // (height of low edge speader - height of april tag)
          Units.inchesToMeters((6 * 12 + 6) - (4 * 12 + 3 + 7 / 8)), // z
          new Rotation3d());

  public AutoShooter(
      Supplier<AprilTag> aprilTagSupplier,
      Shooter shooterSubsystem,
      DriveTrain drivetrainSubsystem,
      BooleanSupplier shooterLoaded) {
    m_aprilTagProvider = aprilTagSupplier;
    m_shooter = shooterSubsystem;
    m_shooterLoaded = shooterLoaded;

    m_aprilAlignCommand =
        new AprilAlignToSpeakerRadiallyCommand(aprilTagSupplier, drivetrainSubsystem);
    m_orientShooterAngleCommand =
        new OrientShooterAngle(shooterSubsystem, this::CalculateShooterAngle);

    m_orientShooterAngleCommand.unless(m_aprilAlignCommand::isFinished);
  }

  @Override
  public void initialize() {
    m_aprilAlignCommand.initialize();
    m_orientShooterAngleCommand.initialize();
  }

  @Override
  public void execute() {
    m_aprilAlignCommand.execute();
    m_orientShooterAngleCommand.execute();
  }

  public double CalculateShooterAngle() {
    Pose3d botToTargetPose = m_aprilTagProvider.get().pose.transformBy(APRILTAG_TO_SHOOTINGTARGET);
    double distanceToTarget = botToTargetPose.getTranslation().toTranslation2d().getNorm();

    Transform3d bufferTransform = CalculateBufferFromDistanceToTarget(distanceToTarget);

    Pose3d botToTargetPoseWithBuffer = botToTargetPose.transformBy(bufferTransform);
    double desiredShooterAngle =
        Math.atan2(
            botToTargetPoseWithBuffer.getZ(),
            botToTargetPoseWithBuffer.getTranslation().toTranslation2d().getNorm());

    return MathUtil.clamp(
        desiredShooterAngle, Shooter.MIN_ANGLE.getRadians(), Shooter.MAX_ANGLE.getRadians());
  }

  protected Transform3d CalculateBufferFromDistanceToTarget(double distanceToTarget) {
    return new Transform3d(
        0, // x
        0, // y
        0, // z
        new Rotation3d()); // TODO: can be tuned/calculated
  }

  @Override
  public void end(boolean interrupted) {
    m_aprilAlignCommand.end(interrupted);
    m_orientShooterAngleCommand.end(interrupted);

    if (!interrupted && m_shooterLoaded.getAsBoolean())
      m_shooter.ShootSpeakerSpeed(); // shoot if we actually reached the target
  }

  @Override
  public boolean isFinished() {
    return m_aprilAlignCommand.isFinished() && m_orientShooterAngleCommand.getController().atGoal();
  }
  
  public interface AutoShooterExplanation{
  
}
}
