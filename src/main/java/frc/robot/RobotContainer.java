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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Other.SmartDashboardSetup;
import frc.robot.Other.SubsystemGetter;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.SwerveDriveCommand;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  SubsystemGetter Get = new SubsystemGetter();
  SmartDashboardSetup SDS = new SmartDashboardSetup();
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
      new SwerveDriveCommand(() -> 1, () -> 0, () -> 0, () -> .3, Get.DriveTrainSub());
  final Command DriveSide =
      new SwerveDriveCommand(() -> 0, () -> 1, () -> 0, () -> .3, Get.DriveTrainSub());
  final Command Rotate =
      new SwerveDriveCommand(() -> 0, () -> 0, () -> .3, () -> 0, Get.DriveTrainSub());
  // commands
  final Command ShootNoteCommandNoWait =
      Get.ShooterSub()
          .ShootSpeakerCommand()
          .alongWith(Get.IntakeWheelsSub().EjectNoteCommand())
          .withTimeout(0.5)
          .withName("Shoot Speaker No Wait");

  final Command ShootNoteCommand =
      Get.ShooterSub()
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(Get.ShooterSub()::IsShooterAtSpeakerSpeed)
                  .andThen(Get.IntakeWheelsSub().EjectNoteCommand()))
          .withName("Shoot Speaker When At Target Speed");

  final Command AmpCommand =
      Get.ShooterSub()
          .ShootAmpCommand()
          .alongWith(
              Commands.waitUntil(Get.ShooterSub()::IsShooterAtAmpSpeed)
                  .andThen(Get.IntakeWheelsSub().EjectNoteCommand()))
          .withName("Shoot Amp When at Target Speed");

  final Command AutoShootNote =
      Get.ShooterSub()
          .ShootSpeakerCommand()
          .alongWith(
              Commands.waitUntil(Get.ShooterSub()::IsShooterAtSpeakerSpeed)
                  .andThen(Get.IntakeWheelsSub().EjectNoteCommand()))
          .withTimeout(1.5) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoOnlyShootNote =
      Get.ShooterSub().ShootSpeakerCommand().withName("Auto Shoot Command");
  final Command AutoIntakeOut =
      Get.IntakeWheelsSub() // shooter speed up
          .EjectNoteCommand()
          .withTimeout(1.0) // 0.5 (shooter) + 0.5 command
          .withName("Auto Shoot Command");
  final Command AutoIntakeNoteCommand =
      Get.IntakeSub()
          .autoIntakeDown()
          .onlyIf(() -> !Get.IntakeWheelsSub().NoteIsLoaded())
          .alongWith(Get.IntakeWheelsSub().IntakeNoteCommand())
          .finallyDo(Get.IntakeSub().autoIntakeUp()::schedule);
  final Command AutoEjectNoteCommand =
      Get.IntakeSub()
          .autoIntakeDown()
          .andThen(Get.IntakeWheelsSub().EjectNoteCommand().withTimeout(0.5));

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;
  // Creating 2d field in Sim/ShuffleBoard
  private final Field2d field;
  // Trying to get feedback from auto
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
    Get.DriveTrainSub().setName("DriveTrain");

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
            Get.IntakeSub(), Get.IntakeWheelsSub(), AutoIntake.DrivingState.DriveIntakeDown));
    NamedCommands.registerCommand(
        "AutoRaiseIntake",
        new AutoIntake(
            Get.IntakeSub(), Get.IntakeWheelsSub(), AutoIntake.DrivingState.DriveIntakeUp));
    NamedCommands.registerCommand("Intake", new AutoIntake(Get.IntakeSub(), Get.IntakeWheelsSub()));
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
    field.setRobotPose(Get.DriveTrainSub().getPose2d());
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
    Get.DriveTrainSub()
        .setDefaultCommand(
            new SwerveDriveCommand(
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                () -> driverController.getRightTriggerAxis(),
                Get.DriveTrainSub(),
                () -> driverController.getLeftTriggerAxis() >= 0.5));

    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    driverController.a().onTrue(Get.DriveTrainSub().toggleFieldRelativeEnable());
    driverController.b().onTrue(Get.DriveTrainSub().ZeroGyro());
    driverController
        .start()
        .onTrue(Get.DriveTrainSub().resetPose2d()); // RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(Get.DriveTrainSub().WheelLockCommand()); // lock wheels

    // Manipulator Controller commands
    manipController
        .leftBumper() // Angle down the shooter
        .whileTrue(Get.HangSub().LowerHangAuto());
    manipController
        .rightBumper() // Angle up the shooter
        .whileTrue(Get.HangSub().RaiseHangAuto());

    manipController.start().onTrue(Get.IntakeSub().resetIntakePos());

    manipController
        .a() // Shoot the note
        .whileTrue(ShootNoteCommand.withTimeout(2.5));
    manipController.b().whileTrue(AutoIntakeNoteCommand);
    manipController
        .x() // eject the intake command
        .whileTrue(Get.IntakeWheelsSub().IntakeNoteCommand());

    manipController.povUp().onTrue(Get.IntakeSub().autoIntakeUp());
    manipController.povDown().onTrue(Get.IntakeSub().autoIntakeDown());
    manipController
        .povLeft()
        .whileTrue(Get.IntakeSub().ManualLowerIntakeCommand()); // lower the intake arm
    manipController
        .povRight()
        .whileTrue(Get.IntakeSub().ManualRaiseIntakeCommand()); // raise the intake arm

    manipController
        .y() // eject the intake command
        .whileTrue(Get.IntakeWheelsSub().EjectNoteCommand());
    manipController.rightTrigger(0.5).whileTrue(Get.ShooterSub().ShootSpeakerCommand());
    manipController
        .leftTrigger(0.5)
        .whileTrue(Get.IntakeWheelsSub().IntakeNoteCommandrunRegardless());

    // Debug controller
    // - Manual hanger commands
    debugController
        .leftBumper() // Left Hanger arm down
        .whileTrue(
            Get.HangSub().runEnd(Get.HangSub()::LeftHangDown, Get.HangSub()::LeftHangStop));
    debugController
        .a() // Left Hanger arm up
        .whileTrue(
            Get.HangSub().runEnd(Get.HangSub()::LeftHangUp, Get.HangSub()::LeftHangStop));

    debugController
        .rightBumper() // Right Hanger arm down
        .whileTrue(
            Get.HangSub().runEnd(Get.HangSub()::RightHangDown, Get.HangSub()::RightHangStop));
    debugController
        .b() // Right Hanger arm up
        .whileTrue(
            Get.HangSub().runEnd(Get.HangSub()::RightHangUp, Get.HangSub()::RightHangStop));

    debugController.povUp().whileTrue(Get.ShooterSub().ManualAngleUp());
    debugController.x().whileTrue(DriveForward);
    debugController.povDown().whileTrue(Get.ShooterSub().ManualAngleDown());
    debugController.rightTrigger(.5).whileTrue(Get.IntakeWheelsSub().ReIntakeNoteCommand());
  }

  private void configureShuffleboard() {
    SDS.SmartDashboardConfig();
  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file

    Command autoCommand = autoChooser.getSelected();

    return autoCommand.beforeStarting(
        () -> Get.DriveTrainSub().resetPose(new Pose2d(1.27, 5.55, new Rotation2d())));
  }
}
