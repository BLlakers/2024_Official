package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Creates our objects from our methods for our classes

  private DriveTrain m_DriveTrain = new DriveTrain(Constants.defaultRobotVersion);
  private Intake m_Intake = new Intake();
  private Hanger m_Hanger = new Hanger();
  private Limelight m_Limelight = new Limelight();
  private Shooter m_Shooter = new Shooter();
 

  public DriveTrain GetDriveTrainSub() {
    return m_DriveTrain;
  }

  public Hanger GetHangSub() {
    return m_Hanger;
  }

  public Limelight GetLimelightSub() {
    return m_Limelight;
  }

  public Shooter GetShootWerSub() {
    return m_Shooter;
  }

  public Intake GetIntakeSub() {
    return m_Intake;
  }

  // Shooter

  /**
   * Creates buttons and controller for: - the driver controller (port 0) - the manipulator
   * controller (port 1) - the debug controller (port 2)
   */
  CommandXboxController driverController =
      new CommandXboxController(Constants.Controller.DriverControllerChannel);

  CommandXboxController manipController =
      new CommandXboxController(Constants.Controller.ManipControllerChannel);
  CommandXboxController debugController =
      new CommandXboxController(Constants.Controller.DebugControllerChannel);
  final Command DriveForward =
      new SwerveDriveCommand(() -> 1, () -> 0, () -> 0, () -> .3, m_DriveTrain);
  final Command DriveBack =
      new SwerveDriveCommand(() -> -1, () -> 0, () -> 0, () -> .3, m_DriveTrain);
  final Command DriveRight =
      new SwerveDriveCommand(() -> 0, () -> 1, () -> 0, () -> .3, m_DriveTrain);
  final Command DriveLeft =
      new SwerveDriveCommand(() -> 0, () -> -1, () -> 0, () -> .3, m_DriveTrain);
  final Command RotateRight =
      new SwerveDriveCommand(() -> 0, () -> 0, () -> .3, () -> 0, m_DriveTrain);
  final Command RotateLeft =
      new SwerveDriveCommand(() -> 0, () -> 0, () -> .3, () -> 0, m_DriveTrain);
  // commands
  final Command ShootNoteCommandNoWait =
      m_Shooter
          .ShootSpeakerCommand()
          .alongWith(m_Intake.GetIntakeWheels().EjectNoteCommand())
          .withTimeout(0.5)
          .withName("Shoot Speaker No Wait");

  final Command ShootNoteCommand =
      m_Shooter
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(m_Shooter::IsShooterAtSpeakerSpeed)
                  .andThen(m_Intake.GetIntakeWheels().EjectNoteCommand()))
          .withName("Shoot Speaker When At Target Speed");

  final Command AmpCommand =
      m_Shooter
          .ShootAmpCommand()
          .alongWith(
              Commands.waitUntil(m_Shooter::IsShooterAtAmpSpeed)
                  .andThen(m_Intake.GetIntakeWheels().EjectNoteCommand()))
          .withName("Shoot Amp When at Target Speed");

  final Command AutoShootNote =
      m_Shooter
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(m_Shooter::IsShooterAtSpeakerSpeed)
                  .andThen(m_Intake.GetIntakeWheels().EjectNoteCommand()))
          .withTimeout(1.5) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoOnlyShootNote = m_Shooter.ShootSpeakerCommand().withName("Auto Shoot Command");
  final Command AutoIntakeOut =
      m_Intake
          .GetIntakeWheels() // shooter speed up
          .EjectNoteCommand()
          .withTimeout(1.0) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoIntakeNoteCommand =
      m_Intake
          .autoIntakeDown()
          .onlyIf(() -> !m_Intake.NoteIsLoaded())
          .alongWith(m_Intake.GetIntakeWheels().IntakeNoteCommand())
          .finallyDo(m_Intake.autoIntakeUp()::schedule);
  final Command AutoEjectNoteCommand =
      m_Intake
          .autoIntakeDown()
          .andThen(m_Intake.GetIntakeWheels().EjectNoteCommand().withTimeout(0.5));

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;
  // Creating 2d field in Sim/ShuffleBoard
  private final Field2d field;
  // Trying to get feedback from auto
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
    m_DriveTrain.setName("DriveTrain");

    configureShuffleboard();
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    NamedCommands.registerCommand("Shoot", AutoShootNote);
    NamedCommands.registerCommand("ShootOnly", AutoOnlyShootNote);
    NamedCommands.registerCommand("IntakeOut", AutoIntakeOut);
    NamedCommands.registerCommand("ShootNoDelay", ShootNoteCommandNoWait);
    NamedCommands.registerCommand(
        "AutoLowerIntake",
        new AutoIntake(
            m_Intake, m_Intake.GetIntakeWheels(), AutoIntake.DrivingState.DriveIntakeDown));
    NamedCommands.registerCommand(
        "AutoRaiseIntake",
        new AutoIntake(
            m_Intake, m_Intake.GetIntakeWheels(), AutoIntake.DrivingState.DriveIntakeUp));
    NamedCommands.registerCommand("Intake", new AutoIntake(m_Intake, m_Intake.GetIntakeWheels()));
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name:
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Creates a field to be put to the shuffleboard
    field = new Field2d();

    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });
  }

  public void periodic() {
    // us trying to set pose for field2d
    field.setRobotPose(m_DriveTrain.getPose2d());
  }

  /**
   * Creates Command Bindings. Read description down below:
   *
   * <p>Use this method to define your trigger->comand mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /**
     * Swerve Drive Controller Command
     *
     * <p>Controls: - Left Stick: Steering - Right Stick: Rotate the robot - Right Trigger: provide
     * gas - Left Trigger: reduce maximum driving speed by 50% RECOMMENDED TO USE
     */
    m_DriveTrain.setDefaultCommand(
        new SwerveDriveCommand(
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> driverController.getRightTriggerAxis(),
            m_DriveTrain,
            () -> driverController.getLeftTriggerAxis() >= 0.5));

    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    driverController.a().onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    driverController.b().onTrue(m_DriveTrain.ZeroGyro());
    driverController.start().onTrue(m_DriveTrain.resetPose2d()); // RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(m_DriveTrain.WheelLockCommand()); // lock wheels

    // Manipulator Controller commands
    manipController
        .leftBumper() // Angle down the shooter
        .whileTrue(m_Hanger.LowerHangAuto());
    manipController
        .rightBumper() // Angle up the shooter
        .whileTrue(m_Hanger.RaiseHangAuto());

    manipController.start().onTrue(m_Intake.resetIntakePos());

    manipController
        .a() // Shoot the note
        .whileTrue(ShootNoteCommand.withTimeout(2.5));
    manipController.b().whileTrue(AutoIntakeNoteCommand);
    manipController
        .x() // eject the intake command
        .whileTrue(m_Intake.GetIntakeWheels().IntakeNoteCommand());

    manipController.povUp().onTrue(m_Intake.autoIntakeUp());
    manipController.povDown().onTrue(m_Intake.autoIntakeDown());
    manipController
        .povLeft()
        .whileTrue(m_Intake.ManualLowerIntakeCommand()); // lower the intake arm
    manipController
        .povRight()
        .whileTrue(m_Intake.ManualRaiseIntakeCommand()); // raise the intake arm

    manipController
        .y() // eject the intake command
        .whileTrue(m_Intake.GetIntakeWheels().EjectNoteCommand());
    manipController.rightTrigger(0.5).whileTrue(m_Shooter.ShootSpeakerCommand());
    manipController
        .leftTrigger(0.5)
        .whileTrue(m_Intake.GetIntakeWheels().IntakeNoteCommandrunRegardless());

    // Debug controller
    // - Manual hanger commands
    debugController
        .leftBumper() // Left Hanger arm down
        .whileTrue(m_Hanger.runEnd(m_Hanger::LeftHangDown, m_Hanger::LeftHangStop));
    debugController
        .a() // Left Hanger arm up
        .whileTrue(m_Hanger.runEnd(m_Hanger::LeftHangUp, m_Hanger::LeftHangStop));

    debugController
        .rightBumper() // Right Hanger arm down
        .whileTrue(m_Hanger.runEnd(m_Hanger::RightHangDown, m_Hanger::RightHangStop));
    debugController
        .b() // Right Hanger arm up
        .whileTrue(m_Hanger.runEnd(m_Hanger::RightHangUp, m_Hanger::RightHangStop));

    debugController.povUp().whileTrue(m_Shooter.ManualAngleUp());
    debugController.x().whileTrue(DriveForward);
    debugController.povDown().whileTrue(m_Shooter.ManualAngleDown());
    debugController.rightTrigger(.5).whileTrue(m_Intake.GetIntakeWheels().ReIntakeNoteCommand());
  }

  private void configureShuffleboard() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Add subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(m_DriveTrain.getName() + "/Reset Pose 2D", m_DriveTrain.resetPose2d());

    SmartDashboard.putData(m_Shooter);
    SmartDashboard.putData(m_Hanger);
    SmartDashboard.putData(m_Intake);
    SmartDashboard.putData(m_Intake.GetIntakeWheels());
    SmartDashboard.putData(m_Limelight);
  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file

    Command autoCommand = autoChooser.getSelected();

    return autoCommand.beforeStarting(
        () -> m_DriveTrain.resetPose(new Pose2d(1.27, 5.55, new Rotation2d())));
  }
}
