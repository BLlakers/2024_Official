package frc.robot.commands;

// import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AprilAlignToSpeakerRadiallyCommand extends Command {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, 2); // TODO DO 1 PID AT A TIME !!!!!
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, 2); // TODO DO 1 PID AT A TIME !!!!!
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(60), 8); // TODO DO 1 PID AT A TIME !!!!!

  private static final double MIN_RADIUS = 2.1; // meters
  private static final double OPTIMAL_RADIUS = 2.25; // meters
  private static final double MAX_RADIUS = 2.5; // meters

  private final DriveTrain m_drivetrain;
  private final Supplier<AprilTag> m_aprilTagProvider;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.0, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1.0, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(0.8, 0, 0.0, OMEGA_CONSTRAINTS);

  private Pose2d goalPose;

  public AprilAlignToSpeakerRadiallyCommand(
      Supplier<AprilTag> aprilTagSupplier, DriveTrain drivetrainSubsystem) {
    this.m_drivetrain = drivetrainSubsystem;
    this.m_aprilTagProvider = aprilTagSupplier;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    goalPose = null;
    var robotPose = m_drivetrain.getPose2d();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    // Grab the current states: april tag in view and the current robot pose
    Pose2d robotPose = m_drivetrain.getPose2d();
    AprilTag aprilTag = m_aprilTagProvider.get();
    if (aprilTag.ID <= 0) {
      // is valid if > 0: we update our current estimate of where the april tag is relative to the
      // robot
      m_drivetrain.stopModules();
      return;
    }
    // Find the tag we want to chase
    Pose2d Bot2Tag = aprilTag.pose.toPose2d();
    Translation2d Bot2Tag_Translation = Bot2Tag.getTranslation();
    Rotation2d targetDirection = Bot2Tag_Translation.getAngle().plus(Rotation2d.fromDegrees(180));

    // Transform the tag's pose to set our goal
    Pose2d botToGoal =
        new Pose2d(
            Bot2Tag_Translation.times(1 - (OPTIMAL_RADIUS / Bot2Tag_Translation.getNorm())),
            targetDirection);

    goalPose = botToGoal.transformBy(robotPose.minus(new Pose2d()));

    if (null != goalPose) {
      // Drive
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());

      SmartDashboard.putNumber(
          m_drivetrain.getName() + "/AprilAlignCommand/Goal/X", goalPose.getX());
      SmartDashboard.putNumber(
          m_drivetrain.getName() + "/AprilAlignCommand/Goal/Y", goalPose.getY());
      SmartDashboard.putNumber(
          m_drivetrain.getName() + "/AprilAlignCommand/Goal/Rot",
          goalPose.getRotation().getDegrees());
    }

    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/VelX", xSpeed);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/VelY", ySpeed);
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/AprilAlignCommand/Command/VelRot", omegaSpeed);

    m_drivetrain.driveChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }

  protected boolean PoseIsWithinGoalRadius(Pose2d robotPose) {
    Translation2d goalLocation =
        new Translation2d(xController.getGoal().position, yController.getGoal().position);
    double distance = robotPose.getTranslation().getDistance(goalLocation);

    return (MIN_RADIUS <= distance) && (distance <= MAX_RADIUS);
  }

  public boolean RobotIsWithinGoalRadius() {
    return PoseIsWithinGoalRadius(m_drivetrain.getPose2d());
  }

  @Override
  public boolean isFinished() {
    return omegaController.atGoal() && RobotIsWithinGoalRadius();
  }
/** <b> DETAILED EXPLANATION </b>
   * 
   * 
   * 
   * 
   * 
   */
  public static int Explanation() {
    return 1;
  }
}
