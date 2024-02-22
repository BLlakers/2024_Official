package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.AprilAlignCommand;
import frc.robot.subsystems.Limelight;
//add in later
//import frc.robot.commands.AprilAlignCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  DriveTrain m_DriveTrain = new DriveTrain(Constants.defaultRobotVersion);
  Limelight m_Stuff = new Limelight();
  Intake m_Intake = new Intake();
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

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;
  // Creating 2d field in Sim/ShuffleBoard
  private final Field2d field;
  // Trying to get feedback from auto
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
    configureBindings();
    // Build an auto chooser. How we choose which auto we want to run.
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /// new Trigger(m_exampleSubsystem::exampleCondition)
    // BINDINGS OF EACH COMMAND
    //LIMELIGHT
    driverButtonLeft.whileTrue(new AprilAlignCommand(() -> m_Stuff.getCurrentAprilTag(), m_DriveTrain));
    
    //SWERVE DRIVE/ DRIVETRAIN
    m_DriveTrain.setDefaultCommand(new SwerveDriveCommand(() -> driverController.getLeftY(),
        () -> driverController.getLeftX(), () -> driverController.getRightX(),
        () -> driverController.getRightTriggerAxis(), m_DriveTrain));
    // WHEEL LOCK
    driverButtonRS.onTrue(m_DriveTrain.WheelLockCommand());
    // ZERO GYRO
    driverButtonB.onTrue(m_DriveTrain.ZeroGyro());
    // TOGGLE FIELD RELATIVE ON AND OFF
    driverButtonA.onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    
    // RESETING OUR POSE 2d/ odometry
    driverButtonOptions.onTrue(m_DriveTrain.resetPose2d());
    
    driverButtonY.whileTrue(m_Shooter.RunShooter());
    driverButtonY.whileFalse(m_Shooter.StopShooter());

    manipButtonB.whileTrue(m_Intake.RunIntakeWheels());
    manipButtonB.whileFalse(m_Intake.StopIntakeWheels());

    driverButtonLeft.whileTrue(m_Shooter.AngleDownShooter());// moves down
    driverButtonLeft.onFalse(m_Shooter.AngleStop());
    driverButtonRight.whileTrue(m_Shooter.AngleUpShooter()); // moves up
    driverButtonRight.onFalse(m_Shooter.AngleStop());

    manipButtonLeft.whileTrue(m_Intake.LowerIntake());
    manipButtonRight.whileTrue(m_Intake.RaiseIntake());
    manipButtonLeft.onFalse(m_Intake.StopIntake());
    manipButtonRight.onFalse(m_Intake.StopIntake());

    manipButtonX.whileTrue(m_Hanger.LeftHangUp());
    manipButtonY.whileTrue(m_Hanger.RightHangUp());

  }
  public Command getAutonomousCommand() {
    // loads New Auto auto file
    // Where we set order of autoCommands
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_DriveTrain.resetPose(new Pose2d(1.00, 5.00, new Rotation2d(0)))),
        new WaitCommand(3.0),
        autoChooser.getSelected());

  }
}