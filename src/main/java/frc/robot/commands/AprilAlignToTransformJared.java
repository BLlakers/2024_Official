package frc.robot.commands;

// import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AprilAlignToTransformJared extends Command {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 3);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 3);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(60), 8);

  private final ProfiledPIDController m_xController =
      new ProfiledPIDController(1, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController =
      new ProfiledPIDController(1, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_rotController =
      new ProfiledPIDController(.5, 0, 0.0, OMEGA_CONSTRAINTS);

  public static final Transform2d TRANSFORM_HANGER_LEFT =
      new Transform2d(
          Units.inchesToMeters(12 + 4 + 5 / 8),
          Units.inchesToMeters(85.9 / 2)
              - Math.abs(Constants.Drive.SMFrontRightLocation.getX() - 6),
          new Rotation2d());
  public static final Transform2d TRANSFORM_HANGER_RIGHT =
      new Transform2d(
          TRANSFORM_HANGER_LEFT.getX(),
          -TRANSFORM_HANGER_LEFT
              .getY(), // Inverts the y-direction alignment to get right hanger position
          TRANSFORM_HANGER_LEFT.getRotation());

  public static final Transform2d TRANSFORM_SPEAKER_FRONT =
      new Transform2d(Units.inchesToMeters(3 * 12 + 1 / 2), 0, new Rotation2d());
  public static final Transform2d TRANSFORM_SPEAKER_LEFT =
      TRANSFORM_SPEAKER_FRONT.plus(
          new Transform2d(0, 0, Rotation2d.fromDegrees(60))); // rotate by 60 degrees
  public static final Transform2d TRANSFORM_SPEAKER_RIGHT =
      TRANSFORM_SPEAKER_FRONT.plus(
          new Transform2d(0, 0, Rotation2d.fromDegrees(-60))); // rotate by 60 degrees

  // subsystems
  private DriveTrain m_drivetrain;
  private Supplier<AprilTag> m_aprilTagProvider;

  // fields for tracking
  private Pose2d m_goalPose;
  private Pose2d m_tagToGoal;

  public AprilAlignToTransformJared(
      Supplier<AprilTag> aprilTagSupplier,
      DriveTrain drivetrainSubsystem,
      Pose2d
          goalTransformRelativeToAprilTag) { // GoalTransformTo tag is where we want to stop. Should
    // be a negative number.
    this.m_drivetrain = drivetrainSubsystem;
    this.m_aprilTagProvider = aprilTagSupplier;
    this.m_tagToGoal = goalTransformRelativeToAprilTag;

    m_xController.setTolerance(0.1);
    m_yController.setTolerance(0.1);
    m_rotController.setTolerance(Units.degreesToRadians(3));
    m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    m_goalPose = null;
    var robotPose = m_drivetrain.getPose2d();
    m_rotController.reset(robotPose.getRotation().getRadians());
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    // Grab the current states: april tag in view and the current robot pose
    Pose2d robotPose = m_drivetrain.getPose2d();
    AprilTag aprilTag = m_aprilTagProvider.get();
    if (aprilTag.ID
        <= 0) { // is valid if > 0: we update our current estimate of where the april tag is
      // relative to the robot
      m_drivetrain.stopModules();
      return;
    }
    // Find the tag we want to chase
    Pose2d BotToTag = aprilTag.pose.toPose2d(); // get pose rel to tag

    Translation2d Bot2Tag_Translation = BotToTag.getTranslation(); // get translation
    Rotation2d TagDirection = Bot2Tag_Translation.getAngle(); // get rot of translation



    Rotation2d TESTINGTagDir = BotToTag.getRotation();
    Pose2d botToGoalPose =
        new Pose2d(
            Bot2Tag_Translation.minus(m_tagToGoal.getTranslation()), // our bot to tag pose minus our transform pose 
            TagDirection.rotateBy(m_tagToGoal.getRotation())); //  our bot to tag rot plus our provided goal rotation

    SmartDashboard.putNumber(m_drivetrain.getName() + "/Testing/Bot2Tag/X", BotToTag.getX());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/Testing/Bot2Tag/Y", BotToTag.getY());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/Bot2Tag/Omega", BotToTag.getRotation().getDegrees());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/botToGoal/X", botToGoalPose.getTranslation().getX());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/botToGoal/Y", botToGoalPose.getTranslation().getY());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/botToGoal/Omega",
        botToGoalPose.getRotation().getDegrees());
     SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/botToGoal/LimelightTagDir",
        TagDirection.getDegrees());   
    // m_goalPose = robotPose.transformBy(botToGoalPose);
    m_goalPose = robotPose.transformBy(botToGoalPose.minus(new Pose2d())); // Bot to goal pose plus our robot pose 

    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/goalPose/X", m_goalPose.getTranslation().getX());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/goalPose/Y", m_goalPose.getTranslation().getY());
    SmartDashboard.putNumber(
        m_drivetrain.getName() + "/Testing/goalPose/Omega", m_goalPose.getRotation().getDegrees());

    if (null != m_goalPose) {
      // Drive
      m_xController.setGoal(m_goalPose.getX());
      m_yController.setGoal(0);
      m_rotController.setGoal(0);
    }

    //double xSpeed = m_xController.calculate(robotPose.getX());
    //if (m_xController.atGoal()) {
    //  xSpeed = 0;
   // }

    double ySpeed = m_yController.calculate(robotPose.getY());
    if (m_yController.atGoal()) {
      ySpeed = 0;
    }

    double rotSpeed = m_rotController.calculate(robotPose.getRotation().getRadians());
    System.out.println(rotSpeed);
    if (m_rotController.atGoal()) {
      rotSpeed = 0;
    }

    m_drivetrain.driveChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, ySpeed, rotSpeed, robotPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    return m_rotController.atGoal() && m_xController.atGoal() && m_yController.atGoal();
  }
  
}

