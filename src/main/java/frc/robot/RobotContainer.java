package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  DriveTrain m_DriveTrain = new DriveTrain(Constants.defaultRobotVersion);
  Limelight m_Limelight = new Limelight();
  Intake m_Intake = new Intake();
  IntakeWheels m_IntakeWheels = new IntakeWheels();
  Shooter m_Shooter = new Shooter();
  Hanger m_Hanger = new Hanger();
  // Shooter

  /**
   * Creates buttons and controller for:
   * - the driver controller (port 0)
   * - the manipulator controller (port 1)
   * - the debug controller (port 2)
   */

  CommandXboxController driverController = new CommandXboxController(Constants.Controller.DriverControllerChannel);
  CommandXboxController manipController = new CommandXboxController(Constants.Controller.ManipControllerChannel);
  XboxController manipControllerTest = new XboxController(Constants.Controller.ManipControllerChannel);
  CommandXboxController debugController = new CommandXboxController(Constants.Controller.DebugControllerChannel);
  JoystickButton manipbButton = new JoystickButton(manipControllerTest, 0);
  // commands
  final Command ShootNoteCommand = m_Shooter.RunShooter()
      .andThen(new WaitCommand(0.5))
      .andThen(m_IntakeWheels.ReverseIntakeWheelsCommand())
      .andThen(new WaitCommand(2.0))
      .finallyDo(
          () -> {
            m_Shooter.StopShooter().alongWith(m_IntakeWheels.StopIntakeWheelsCommand()).schedule();
          })
      .withName("Shoot Command");

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
    NamedCommands.registerCommand("Shoot", ShootNoteCommand);
    NamedCommands.registerCommand("AutoLowerIntake",
        new AutoIntake(m_Intake, m_IntakeWheels, AutoIntake.DrivingState.DriveIntakeDown));
    NamedCommands.registerCommand("AutoRaiseIntake",
        new AutoIntake(m_Intake, m_IntakeWheels, AutoIntake.DrivingState.DriveIntakeUp));
    NamedCommands.registerCommand("Intake",
        new AutoIntake(m_Intake, m_IntakeWheels));
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name:
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Creates a field to be put to the shuffleboard
    field = new Field2d();

    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
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
   * <p>
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
    /**
     * Swerve Drive Controller Command
     * 
     * Controls:
     * - Left Stick: Steering
     * - Right Stick: Rotate the robot
     * - Right Trigger: provide gas
     * - Left Trigger: reduce maximum driving speed by 50% RECOMMENDED TO USE
     */
    /*
     * m_DriveTrain.setDefaultCommand(new SwerveDriveCommand(() ->
     * driverController.getLeftY(),
     * () -> driverController.getLeftX(), () -> driverController.getRightX(),
     * () -> driverController.getRightTriggerAxis(), m_DriveTrain,
     * () -> driverController.getLeftTriggerAxis() >= 0.5));
     */
    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    driverController.a().onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    driverController.b().onTrue(m_DriveTrain.ZeroGyro());
    driverController.start().onTrue(m_DriveTrain.resetPose2d());// RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(m_DriveTrain.WheelLockCommand()); // lock wheels
    // Manipulator Controller commands
    // - Intake commands
    /*
     * manipController.b() // Run the intake wheels for intaking a note
     * .whileTrue(m_IntakeWheels.RunIntakeWheelsCommand())
     * .whileFalse(m_IntakeWheels.StopIntakeWheelsCommand());
     */
    manipController.leftStick() // eject the intake command
        .whileTrue(m_IntakeWheels.ReverseIntakeWheelsCommand())
        .whileFalse(m_IntakeWheels.StopIntakeWheelsCommand());

    manipController.povLeft() // lower the intake arm
        .whileTrue(m_Intake.LowerIntakeCommand())
        .onFalse(m_Intake.StopIntakeCommand());
    manipController.povRight() // raise the intake arm
        .whileTrue(m_Intake.RaiseIntakeCommand())
        .onFalse(m_Intake.StopIntakeCommand());
    manipController.leftBumper() // Angle down the shooter
        .whileTrue(m_Shooter.AngleDownShooter())
        .onFalse(m_Shooter.AngleStop());
    manipController.rightBumper() // Angle up the shooter
        .whileTrue(m_Shooter.AngleUpShooter())
        .onFalse(m_Shooter.AngleStop());
    manipController.x().onTrue(m_Intake.resetIntakePos());
    // reset the intake encoder position
    manipController.b() // toggle the intake between it's different states
        .whileTrue(new AutoIntake(m_Intake, m_IntakeWheels));
    manipController.povUp().onTrue(m_Intake.autoIntakeUp());
    manipController.povDown().onTrue(m_Intake.autoIntakeDown());

    // - Shooter commands
    manipController.a() // Shoot the note
        .whileTrue(ShootNoteCommand)
        .whileFalse(m_Shooter.StopShooter());
    // manipController.y().onTrue(new HangCommand(m_Hanger, manipbButton
    // )).onFalse(m_Hanger.HangStopCommand());

    // Debug controller
    // - Manual hanger commands
    debugController.leftBumper() // Left Hanger arm down
        .whileTrue(new InstantCommand(m_Hanger::LeftHangDown, m_Hanger))
        .onFalse(new InstantCommand(m_Hanger::LeftHangStop, m_Hanger));
    debugController.a() // Left Hanger arm up
        .whileTrue(new InstantCommand(m_Hanger::LeftHangUp, m_Hanger))
        .onFalse(new InstantCommand(m_Hanger::LeftHangStop, m_Hanger));

    debugController.rightBumper() // Right Hanger arm down
        .whileTrue(new InstantCommand(m_Hanger::RightHangDown, m_Hanger))
        .onFalse(new InstantCommand(m_Hanger::RightHangStop, m_Hanger));
    debugController.b() // Right Hanger arm up
        .whileTrue(new InstantCommand(m_Hanger::RightHangUp, m_Hanger))
        .onFalse(new InstantCommand(m_Hanger::RightHangStop, m_Hanger));

  }

  private void configureShuffleboard() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    
    // Add subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData("DriveTrain/Reset Pose 2D", m_DriveTrain.resetPose2d());

    SmartDashboard.putData(m_Shooter);
    SmartDashboard.putData(m_Hanger);
    SmartDashboard.putData(m_Intake);
    SmartDashboard.putData(m_IntakeWheels);
    SmartDashboard.putData(m_Limelight);

  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file
    // return new PathPlannerAuto("New Auto");
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_DriveTrain.resetPose(new Pose2d(1.00, 5.00, new Rotation2d(0)))),
        new WaitCommand(3.0),
        autoChooser.getSelected());

  }
}