package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPID;

public class AprilAlignCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0),
            Rotation2d.fromDegrees(180));

    private static final double kdriveMaxDriveSpeed = 0.1; // meters per second

    private final DriveTrainPID m_drivetrain;
    private final Supplier<AprilTag> m_aprilTagProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

    private Pose2d goalPose;

    public AprilAlignCommand(Supplier<AprilTag> aprilTagSupplier, DriveTrainPID drivetrainSubsystem) {
        this.m_drivetrain = drivetrainSubsystem;
        this.m_aprilTagProvider = aprilTagSupplier;

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-1, 1);

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
    System.out.println("Pose Supplier is " + m_aprilTagProvider.get());
    AprilTag aprilTag  = m_aprilTagProvider.get();
    if (aprilTag.ID <= 0) { // is valid if > 0: we update our current estimate of where the april tag is relative to the robot
      m_drivetrain.stopModules();
      return;
    }
    // Find the tag we want to chase
    Pose3d camToTarget = aprilTag.pose;
    Transform2d transform = new Transform2d(
        camToTarget.getTranslation().toTranslation2d(),
        camToTarget.getRotation().toRotation2d());
    
    // Transform the robot's pose to find the tag's pose
    Transform3d robotToCamera3d = Constants.CAMERA_TO_ROBOT.inverse();
    Transform2d robotToCamera2d = new Transform2d(robotToCamera3d.getTranslation().toTranslation2d(),
    robotToCamera3d.getRotation().toRotation2d());
    Pose2d cameraPose = robotPose.transformBy(robotToCamera2d);
    Pose2d targetPose = cameraPose.transformBy(transform);
    
    // Transform the tag's pose to set our goal
    goalPose = targetPose.transformBy(TAG_TO_GOAL);

    if (null != goalPose) {
      // Drive
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());
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

    xSpeed = Math.min(xSpeed, kdriveMaxDriveSpeed);
    ySpeed = Math.min(ySpeed, kdriveMaxDriveSpeed);

    m_drivetrain.driveChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    
  }
 
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopModules();
    }

}