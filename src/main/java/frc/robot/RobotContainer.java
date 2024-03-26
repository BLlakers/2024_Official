package frc.robot;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Other.SubsystemGetter;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.SwerveDriveCommand;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  SubsystemGetter Sub = new SubsystemGetter();

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
      new SwerveDriveCommand(() -> 1, () -> 0, () -> 0, () -> .3, Sub.GetDriveTrain());
  final Command DriveSide =
      new SwerveDriveCommand(() -> 0, () -> 1, () -> 0, () -> .3, Sub.GetDriveTrain());
  final Command Rotate = new SwerveDriveCommand(() -> 0, () -> 0, () -> .3, () -> 0, Sub.GetDriveTrain());
  // commands
  final Command ShootNoteCommandNoWait =
      Sub.GetShooter()
          .ShootSpeakerCommand()
          .alongWith(Sub.GetIntakeWheels().EjectNoteCommand())
          .withTimeout(0.5)
          .withName("Shoot Speaker No Wait");

  final Command ShootNoteCommand =
      Sub.GetShooter()
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(Sub.GetShooter()::IsShooterAtSpeakerSpeed)
                  .andThen(Sub.GetIntakeWheels().EjectNoteCommand()))
          .withName("Shoot Speaker When At Target Speed");

  final Command AmpCommand =
Sub.GetShooter()
          .ShootAmpCommand()
          .alongWith(
              Commands.waitUntil(Sub.GetShooter()::IsShooterAtAmpSpeed)
                  .andThen(Sub.GetIntakeWheels().EjectNoteCommand()))
          .withName("Shoot Amp When at Target Speed");

  final Command AutoShootNote =
      Sub.GetShooter()
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(Sub.GetShooter()::IsShooterAtSpeakerSpeed)
                  .andThen(Sub.GetIntakeWheels().EjectNoteCommand()))
          .withTimeout(1.5) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoOnlyShootNote = Sub.GetShooter().ShootSpeakerCommand().withName("Auto Shoot Command");
  final Command AutoIntakeOut =
      Sub.GetIntakeWheels() // shooter speed up
          .EjectNoteCommand()
          .withTimeout(1.0) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoIntakeNoteCommand =
      Sub.GetIntake()
      .autoIntakeDown()
          .onlyIf(() -> !Sub.GetIntakeWheels().NoteIsLoaded())
          .alongWith(Sub.GetIntakeWheels().IntakeNoteCommand())
          .finallyDo(Sub.GetIntake().autoIntakeUp()::schedule);
  final Command AutoEjectNoteCommand =
      Sub.GetIntake()
          .autoIntakeDown()
          .andThen(Sub.GetIntakeWheels().EjectNoteCommand().withTimeout(0.5));

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;
  // Creating 2d field in Sim/ShuffleBoard
  private final Field2d field;
  // Trying to get feedback from auto
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
    Sub.GetDriveTrain().setName("DriveTrain");

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
            Sub.GetIntake(), Sub.GetIntakeWheels(), AutoIntake.DrivingState.DriveIntakeDown));
    NamedCommands.registerCommand(
        "AutoRaiseIntake",
        new AutoIntake(
            Sub.GetIntake(), Sub.GetIntakeWheels(), AutoIntake.DrivingState.DriveIntakeUp));
    NamedCommands.registerCommand("Intake", new AutoIntake(Sub.GetIntake(), Sub.GetIntakeWheels()));
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
    field.setRobotPose(Sub.GetDriveTrain().getPose2d());
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
    Sub.GetDriveTrain().setDefaultCommand(
        new SwerveDriveCommand(
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> driverController.getRightTriggerAxis(),
            Sub.GetDriveTrain(),
            () -> driverController.getLeftTriggerAxis() >= 0.5));

    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    driverController.a().onTrue(Sub.GetDriveTrain().toggleFieldRelativeEnable());
    driverController.b().onTrue(Sub.GetDriveTrain().ZeroGyro());
    driverController.start().onTrue(Sub.GetDriveTrain().resetPose2d()); // RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(Sub.GetDriveTrain().WheelLockCommand()); // lock wheels

    // Manipulator Controller commands
    manipController
        .leftBumper() // Angle down the shooter
        .whileTrue(Sub.GetHanger().LowerHangAuto());
    manipController
        .rightBumper() // Angle up the shooter
        .whileTrue(Sub.GetHanger().RaiseHangAuto());

    manipController.start().onTrue(Sub.GetIntake().resetIntakePos());

    manipController
        .a() // Shoot the note
        .whileTrue(ShootNoteCommand.withTimeout(2.5));
    manipController.b().whileTrue(AutoIntakeNoteCommand);
    manipController
        .x() // eject the intake command
        .whileTrue(Sub.GetIntakeWheels().IntakeNoteCommand());

    manipController.povUp().onTrue(Sub.GetIntake().autoIntakeUp());
    manipController.povDown().onTrue(Sub.GetIntake().autoIntakeDown());
    manipController
        .povLeft()
        .whileTrue(Sub.GetIntake().ManualLowerIntakeCommand()); // lower the intake arm
    manipController
        .povRight()
        .whileTrue(Sub.GetIntake().ManualRaiseIntakeCommand()); // raise the intake arm

    manipController
        .y() // eject the intake command
        .whileTrue(Sub.GetIntakeWheels().EjectNoteCommand());
    manipController.rightTrigger(0.5).whileTrue(Sub.GetShooter().ShootSpeakerCommand());
    manipController
        .leftTrigger(0.5)
        .whileTrue(Sub.GetIntakeWheels().IntakeNoteCommandrunRegardless());

    // Debug controller
    // - Manual hanger commands
    debugController
        .leftBumper() // Left Hanger arm down
        .whileTrue(Sub.GetHanger().runEnd(Sub.GetHanger()::LeftHangDown, Sub.GetHanger()::LeftHangStop));
    debugController
        .a() // Left Hanger arm up
        .whileTrue(Sub.GetHanger().runEnd(Sub.GetHanger()::LeftHangUp, Sub.GetHanger()::LeftHangStop));

    debugController
        .rightBumper() // Right Hanger arm down
        .whileTrue(Sub.GetHanger().runEnd(Sub.GetHanger()::RightHangDown, Sub.GetHanger()::RightHangStop));
    debugController
        .b() // Right Hanger arm up
        .whileTrue(Sub.GetHanger().runEnd(Sub.GetHanger()::RightHangUp, Sub.GetHanger()::RightHangStop));

    debugController.povUp().whileTrue(Sub.GetShooter().ManualAngleUp());
    debugController.x().whileTrue(DriveForward);
    debugController.povDown().whileTrue(Sub.GetShooter().ManualAngleDown());
    debugController.rightTrigger(.5).whileTrue(Sub.GetIntakeWheels().ReIntakeNoteCommand());
  }

  private void configureShuffleboard() {
    Sub.SmartDashboardSetup();
  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file

    Command autoCommand = autoChooser.getSelected();

    return autoCommand.beforeStarting(
        () -> Sub.GetDriveTrain().resetPose(new Pose2d(1.27, 5.55, new Rotation2d())));
  }
}
