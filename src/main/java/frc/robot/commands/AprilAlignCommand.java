package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainPID;

// TODO DO 1 PID AT A TIME !!!!!
// WHAT I MEAN IS DO ROTATION, Y, then X.
public class AprilAlignCommand extends Command {
  public static double ConstraintsConstant = 1; 
  public static double PIDConstant = 16; 
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 2); //TODO DO 1 PID AT A TIME !!!!!
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 2); // TODO DO 1 PID AT A TIME !!!!!
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.degreesToRadians(60), 8); // TODO DO 1 PID AT A TIME !!!!!

    private static final double MIN_RADIUS = 0.75; // meters
    private static final double MAX_RADIUS = 2; // meters
    private static final double OPTIMAL_RADIUS = 1.5; // meters
    private static final Transform2d DEFAULT_TAG_TO_GOAL = new Transform2d(new Translation2d(OPTIMAL_RADIUS, 0),
            Rotation2d.fromDegrees(0)); //180

    private static final double kdriveMaxDriveSpeed = 0.1; // meters per second

    private final DriveTrainPID m_drivetrain;
    private final Supplier<AprilTag> m_aprilTagProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0.0, X_CONSTRAINTS); //2 TODO DO 1 PID AT A TIME !!!!! 4/4
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0.0, Y_CONSTRAINTS); //2 TODO DO 1 PID AT A TIME !!!!! 4/4
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.5, 0, 0.0, OMEGA_CONSTRAINTS); //1 TODO DO 1 PID AT A TIME !!!!! 2/4

    private Pose2d goalPose;

    public AprilAlignCommand(Supplier<AprilTag> aprilTagSupplier, DriveTrainPID drivetrainSubsystem) {
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
    AprilTag aprilTag  = m_aprilTagProvider.get();
    if (aprilTag.ID <= 0) { // is valid if > 0: we update our current estimate of where the april tag is relative to the robot
      m_drivetrain.stopModules();
      return;
    }
    // Find the tag we want to chase
    Pose3d botToTarget = aprilTag.pose;
    Translation2d botToTargetTranslation = botToTarget.getTranslation().toTranslation2d();
    Rotation2d targetDirection = botToTargetTranslation.getAngle();
    
    
    // Transform the tag's pose to set our goal
    Transform2d botToGoalPose = new Transform2d(
      botToTargetTranslation.times(
        1 - (OPTIMAL_RADIUS / botToTargetTranslation.getNorm())
      ),
      targetDirection
    );
    goalPose = robotPose.transformBy(botToGoalPose);

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
    System.out.println(omegaSpeed);
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    m_drivetrain.driveChassisSpeeds(
       ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    
  }
 
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopModules();
    }

    protected boolean PoseIsWithinGoalRadius(Pose2d robotPose)
    {
        Translation2d goalLocation = new Translation2d(xController.getGoal().position, yController.getGoal().position);
        double distance = robotPose.getTranslation().getDistance(goalLocation);

        return (MIN_RADIUS <= distance) && (distance <= MAX_RADIUS);

    }

    public boolean RobotIsWithinGoalRadius()
    {
      return PoseIsWithinGoalRadius(m_drivetrain.getPose2d());
    }

    @Override
    public boolean isFinished()
    {
        return omegaController.atGoal() && RobotIsWithinGoalRadius();
    }

}