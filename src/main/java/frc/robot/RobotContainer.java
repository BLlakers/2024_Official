
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.SwerveAndDriveConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.subsystems.LimelightTags;
import frc.robot.subsystems.Limelight;
//add in later
//import frc.robot.commands.AprilAlignCommand;

public class RobotContainer {
  DriveTrain DriveTrain = new DriveTrain();
  Limelight Limelight = new Limelight();
  LimelightTags LimelightTags = new LimelightTags();

  XboxController driverController = new XboxController(ButtonConstants.DriverControllerChannel);
  XboxController manipController = new XboxController(ButtonConstants.ManipControllerChannel);
  JoystickButton driverButtonB = new JoystickButton(driverController, ButtonConstants.buttonB);
  JoystickButton manipButtonA = new JoystickButton(manipController, ButtonConstants.buttonA);
  JoystickButton driverButtonA = new JoystickButton(driverController, ButtonConstants.buttonA);

  JoystickButton driverButtonRB = new JoystickButton(driverController, ButtonConstants.buttonRight);
  JoystickButton driverButtonLB = new JoystickButton(driverController, ButtonConstants.buttonLeft);
  JoystickButton driverButtonOption = new JoystickButton(driverController, ButtonConstants.buttonOptions);
  // Constants.buttonX);
  JoystickButton driverButtonX = new JoystickButton(driverController, ButtonConstants.buttonX);
  JoystickButton driverButtonRS = new JoystickButton(driverController, ButtonConstants.buttonRS);
  JoystickButton driverButtonLS = new JoystickButton(driverController, ButtonConstants.buttonLS);
  JoystickButton manipButtonB = new JoystickButton(manipController, ButtonConstants.buttonB);
  JoystickButton manipButtonY = new JoystickButton(manipController, ButtonConstants.buttonY);
  JoystickButton manipButtonRB = new JoystickButton(manipController, ButtonConstants.buttonRight);
  JoystickButton manipButtonLB = new JoystickButton(manipController, ButtonConstants.buttonLeft);
  JoystickButton manipButtonOptions = new JoystickButton(manipController, ButtonConstants.buttonOptions);
  JoystickButton driverButtonOptions = new JoystickButton(driverController, ButtonConstants.buttonOptions);
  JoystickButton manipButtonRS = new JoystickButton(manipController, ButtonConstants.buttonRS);
  // A chooser for autonomous commands
  SendableChooser<Integer> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureShuffleboard();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->comand mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DriveTrain.setDefaultCommand(new SwerveDriveCommand(() -> driverController.getLeftY(), () -> driverController.getLeftX(), () -> driverController.getRightX(), DriveTrain));
    driverButtonX.whileTrue(new AlignCommand(DriveTrain, () -> frc.robot.subsystems.Limelight.angle));
    driverButtonRS.onTrue(DriveTrain.WheelzLock());
    driverButtonB.onTrue(DriveTrain.ZeroHeading()); // When the "B" button on the driver controller is clicked, it Reset's our gyro (will be renamed)
    driverButtonA.onTrue(DriveTrain.toggleFieldRelativeEnable()); // When the "A" button on the driver controller is clicked, it turns off field relative mode (might get rid of TODO)
    driverButtonOptions.onTrue(DriveTrain.resetPose2d()); // When the Options Button on the driver controller is clicked, it resets our robots position
    manipButtonLB.onTrue(DriveTrain.LowerArm()); // when LeftBumper pressed on the manipulating controller, it raises our armPosition (renamed variable soon)
    manipButtonRB.onTrue(DriveTrain.RaiseArm()); // when RightBumper pressed on the manipulating controller, it lowers our armPosition (renamed variable soon)
  }

  private void configureShuffleboard() {
    m_chooser.setDefaultOption("Auto 1", 1);
    m_chooser.addOption("Auto 2", 2);
    m_chooser.addOption("Auto 3", 3);
    SmartDashboard.putData(m_chooser);
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig
    (12.1, 8).setKinematics(DriveTrain.m_kinematics); // we don't know our acceleration 
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), List.of(new Translation2d(1,0), new Translation2d(1,-1)), new Pose2d(2, -1, Rotation2d.fromDegrees(180)),trajectoryConfig);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory, DriveTrain::GetPose2d, DriveTrain.m_kinematics, DriveTrain.xController, DriveTrain.yController, DriveTrain.m_thetaController, DriveTrain::setModuleStates, DriveTrain);
  return new SequentialCommandGroup(
    new InstantCommand(() -> DriveTrain.resetPose(trajectory.getInitialPose())), 
    swerveControllerCommand, 
    new InstantCommand(() -> DriveTrain.stopModules())
  );
  }
}