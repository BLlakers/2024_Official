package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  DriveTrain m_DriveTrain = new DriveTrain(Constants.defaultRobotVersion);
  Limelight m_Stuff = new Limelight();
  Intake m_Intake = new Intake();
  IntakeWheels m_IntakeWheels = new IntakeWheels();
  Shooter m_Shooter = new Shooter();
  Hanger m_Hanger = new Hanger();
  // Shooter

  XboxController driverController = new XboxController(Constants.Controller.DriverControllerChannel);
  JoystickButton driverButtonA = new JoystickButton(driverController, Constants.Controller.buttonA);
  JoystickButton driverButtonB = new JoystickButton(driverController, Constants.Controller.buttonB);
  JoystickButton driverButtonX = new JoystickButton(driverController, Constants.Controller.buttonX);
  JoystickButton driverButtonY = new JoystickButton(driverController, Constants.Controller.buttonY);
  JoystickButton driverButtonRight = new JoystickButton(driverController, Constants.Controller.buttonRight);
  JoystickButton driverButtonLeft = new JoystickButton(driverController, Constants.Controller.buttonLeft);
  JoystickButton driverButtonLS = new JoystickButton(driverController, Constants.Controller.buttonLS);
  JoystickButton driverButtonRS = new JoystickButton(driverController, Constants.Controller.buttonRS);
  JoystickButton driverButtonOptions = new JoystickButton(driverController, Constants.Controller.buttonOptions);

  // Creates buttons and controller for the manipulator controller (port 1)
  XboxController manipController = new XboxController(Constants.Controller.ManipControllerChannel);
  JoystickButton manipButtonA = new JoystickButton(manipController, Constants.Controller.buttonA);
  JoystickButton manipButtonB = new JoystickButton(manipController, Constants.Controller.buttonB);
  JoystickButton manipButtonX = new JoystickButton(manipController, Constants.Controller.buttonX);
  JoystickButton manipButtonY = new JoystickButton(manipController, Constants.Controller.buttonY);
  JoystickButton manipButtonRight = new JoystickButton(manipController, Constants.Controller.buttonRight);
  JoystickButton manipButtonLeft = new JoystickButton(manipController, Constants.Controller.buttonLeft);
  JoystickButton manipButtonOptions = new JoystickButton(manipController, Constants.Controller.buttonOptions);
  JoystickButton manipButtonLS = new JoystickButton(manipController, Constants.Controller.buttonLS);
  JoystickButton manipButtonRS = new JoystickButton(manipController, Constants.Controller.buttonRS);

  POVButton DriverpovUp = new POVButton(driverController, 0);
  POVButton DriverpovRight = new POVButton(driverController, 90);
  POVButton DriverpovDown = new POVButton(driverController, 180);
  POVButton DriverpovLeft = new POVButton(driverController, 270);
  POVButton DriverpovUpRight = new POVButton(driverController, 45);
  POVButton DriverpovDownRight = new POVButton(driverController, 135);
  POVButton DriverpovDownLeft = new POVButton(driverController, 225);
  POVButton DriverpovUpLeft = new POVButton(driverController, 315);

  POVButton ManippovUp = new POVButton(manipController, 0);
  POVButton ManippovRight = new POVButton(manipController, 90);
  POVButton ManippovDown = new POVButton(manipController, 180);
  POVButton ManippovLeft = new POVButton(manipController, 270);
  POVButton ManippovUpRight = new POVButton(manipController, 45);
  POVButton ManippovDownRight = new POVButton(manipController, 135);
  POVButton ManippovDownLeft = new POVButton(manipController, 225);
  POVButton ManippovUpLeft = new POVButton(manipController, 315);

  // commands
  final Command ShootNoteCommand = new InstantCommand(m_Shooter::Shoot)
    .andThen(new WaitCommand(0.5))
    .andThen(m_IntakeWheels.ReverseIntakeWheelsCommand())
    .andThen(new WaitCommand(0.25))
    .finallyDo(
       () -> { m_Shooter.StopShooter().alongWith(m_IntakeWheels.StopIntakeWheelsCommand()).schedule(); }
    );

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
    NamedCommands.registerCommand("AutoLowerIntake", new AutoIntake(m_Intake, m_IntakeWheels, AutoIntake.DrivingState.DriveIntakeDown));
    NamedCommands.registerCommand("AutoRaiseIntake", new AutoIntake(m_Intake, m_IntakeWheels, AutoIntake.DrivingState.DriveIntakeUp));
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
    // BINDINGS OF EACH COMMAND
    // LIMELIGHT
    driverButtonLeft.whileTrue(new AprilAlignCommand(() -> m_Stuff.getCurrentAprilTag(), m_DriveTrain));

    // SWERVE DRIVE/ DRIVETRAIN
    m_DriveTrain.setDefaultCommand(new SwerveDriveCommand(() -> driverController.getLeftY(),
        () -> driverController.getLeftX(), () -> driverController.getRightX(),
        () -> driverController.getRightTriggerAxis(), m_DriveTrain));
    // WHEEL LOCK
    driverButtonRS.onTrue(m_DriveTrain.WheelLockCommand());
    driverButtonB.onTrue(m_DriveTrain.ZeroGyro());
    driverButtonA.onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    // RESETING OUR POSE 2d/ odometry
    driverButtonOptions.onTrue(m_DriveTrain.resetPose2d());

    //driverButtonY.whileTrue(m_Shooter.RunShooter());
    //driverButtonY.whileFalse(m_Shooter.StopShooter());
    manipButtonLS.onTrue(new AutoIntake(m_Intake, m_IntakeWheels));
    manipButtonB.whileTrue(m_IntakeWheels.RunIntakeWheelsCommand());
    manipButtonB.whileFalse(m_IntakeWheels.StopIntakeWheelsCommand());
    driverButtonLeft.whileTrue(m_Shooter.AngleDownShooter());// moves down
    driverButtonLeft.onFalse(m_Shooter.AngleStop());
    driverButtonRight.whileTrue(m_Shooter.AngleUpShooter()); // moves up
    driverButtonRight.onFalse(m_Shooter.AngleStop());
    manipButtonLeft.whileTrue(m_Intake.LowerIntakeCommand()); 
    manipButtonRight.whileTrue(m_Intake.RaiseIntakeCommand());
    ManippovUp.onTrue(m_Intake.autoIntakeUp());
    ManippovDown.onTrue(m_Intake.autoIntakeDown());
    manipButtonLeft.onFalse(m_Intake.StopIntakeCommand());
    manipButtonRight.onFalse(m_Intake.StopIntakeCommand());
    manipButtonA.whileTrue(m_Shooter.RunShooter());
    manipButtonA.whileFalse(m_Shooter.StopShooter());
    manipButtonY.whileTrue(m_IntakeWheels.ReverseIntakeWheelsCommand());
    manipButtonY.whileFalse(m_IntakeWheels.StopIntakeWheelsCommand());
    ManippovLeft.onTrue(m_Intake.resetIntakePos());
    // ManippovRight.whileTrue(m_Intake.autoAmp());
    // ManippovRight.whileFalse(m_Intake.StopIntake());
    // Shooter bindings
   // m_Shooter.setDefaultCommand(new OrientShooterAngle(m_Shooter, OrientShooterAngle.s_DefaultAngle));

  }

  private void configureShuffleboard() {
    // Add commands to the shuffleboard
    SmartDashboard.putData(m_DriveTrain.resetPose2d());

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