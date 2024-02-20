
package frc.robot;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.ArrayList;
import java.util.List;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoRotateArmCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.Constants.RobotVersionConstants;
import frc.robot.Other.RobotVersion;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AprilAlignCommand;
import frc.robot.commands.AutoIntakeDown;
import frc.robot.commands.AutoIntakeUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Tags;
import frc.robot.subsystems.Stuff;
import frc.robot.subsystems.SwerveModule;
//add in later
//import frc.robot.commands.AprilAlignCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Stuff;
import frc.robot.subsystems.Tags;

public class RobotContainer {
  DriveTrainPID m_DriveTrainPID = new DriveTrainPID(Constants.defaultRobotVersion);
  Arm m_Arm = new Arm();
  Stuff m_Stuff = new Stuff();
  Tags m_Tags = new Tags();
  Intake m_Intake = new Intake();
  Shooter m_Shooter = new Shooter();
  Hanger m_Hanger = new Hanger();
  //Shooter 

  XboxController driverController = new XboxController(Constants.DriverControllerChannel);
  XboxController manipController = new XboxController(Constants.ManipControllerChannel);
  JoystickButton driverButtonB = new JoystickButton(driverController, Constants.buttonB);
  JoystickButton manipButtonA = new JoystickButton(manipController, Constants.buttonA);
  JoystickButton driverButtonA = new JoystickButton(driverController, Constants.buttonA);

  JoystickButton driverButtonRight = new JoystickButton(driverController, Constants.buttonRight);
  JoystickButton driverButtonLeft = new JoystickButton(driverController, Constants.buttonLeft);
  JoystickButton driverButtonOption = new JoystickButton(driverController, Constants.buttonOptions);
  JoystickButton driverButtonY = new JoystickButton(driverController, Constants.buttonY);
  // Constants.buttonX);
  JoystickButton driverButtonX = new JoystickButton(driverController, Constants.buttonX);
  JoystickButton driverButtonRS = new JoystickButton(driverController, Constants.buttonRS);
  JoystickButton driverButtonLS = new JoystickButton(driverController, Constants.buttonLS);
  JoystickButton manipButtonB = new JoystickButton(manipController, Constants.buttonB);
  JoystickButton manipButtonY = new JoystickButton(manipController, Constants.buttonY);
  JoystickButton manipButtonRight = new JoystickButton(manipController, Constants.buttonRight);
  JoystickButton manipButtonLeft = new JoystickButton(manipController, Constants.buttonLeft);
  
  POVButton DriverpovUp = new POVButton(driverController, 0);
  POVButton DriverpovRight = new POVButton(driverController, 90);
  POVButton DriverpovDown = new POVButton(driverController, 180);
  POVButton DriverpovLeft = new POVButton(driverController, 270);
  POVButton DriverpovUpRight = new POVButton(driverController, 45);
  POVButton DriverpovDownRight = new POVButton(driverController, 135);
  POVButton DriverpovDownLeft = new POVButton(driverController, 225);
  POVButton DriverpovUpLeft = new POVButton(driverController, 315);

 POVButton ManippovUp = new POVButton(        manipController, 0);
  POVButton ManippovRight = new POVButton(    manipController, 90);
  POVButton ManippovDown = new POVButton(     manipController, 180);
  POVButton ManippovLeft = new POVButton(     manipController, 270);
  POVButton ManippovUpRight = new POVButton(  manipController, 45);
  POVButton ManippovDownRight = new POVButton(manipController, 135);
  POVButton ManippovDownLeft = new POVButton( manipController, 225);
  POVButton ManippovUpLeft = new POVButton(   manipController, 315);





  JoystickButton manipButtonOptions = new JoystickButton(manipController, Constants.buttonOptions);
  JoystickButton driverButtonOptions = new JoystickButton(driverController, Constants.buttonOptions);
  JoystickButton manipButtonRS = new JoystickButton(manipController, Constants.buttonRS);
    JoystickButton manipButtonX = new JoystickButton(manipController, Constants.buttonX);

  // A chooser for autonomous commands
  SendableChooser<Integer> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
    configureShuffleboard();
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    

    NamedCommands.registerCommand("AutoLowerIntake", new AutoIntakeDown(m_Intake));
    NamedCommands.registerCommand("AutoRaiseIntake", new AutoIntakeUp(m_Intake));

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
  public void periodic(){
    field.setRobotPose(m_DriveTrainPID.getPose2d());
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /// new Trigger(m_exampleSubsystem::exampleCondition)
    /// .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    /// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
   // m_Intake.setDefaultCommand(new AutoIntake(m_Intake));

   driverButtonLeft.whileTrue(new AprilAlignCommand(() -> m_Stuff.getCurrentAprilTag(), m_DriveTrainPID));
    m_DriveTrainPID.setDefaultCommand(new SwerveDriveCommand(() -> driverController.getLeftY(),
        () -> driverController.getLeftX(), () -> driverController.getRightX(),() -> driverController.getRightTriggerAxis(), m_DriveTrainPID));
    // limelight allign works on both controllers
    // manipButtonX.whileTrue(new AlignCommand(m_DriveTrain, () ->
    // frc.robot.subsystems.Stuff.angle));
    driverButtonX.whileTrue(new AlignCommand(m_DriveTrainPID, () -> frc.robot.subsystems.Stuff.angle));
    // manipButtonB.whileTrue(new AprilAlignCommand(m_DriveTrain, () ->
    // frc.robot.subsystems.Tags.tx2));
    // driverButtonB.whileTrue(new FieldAlignedCommand(m_DriveTrain));
    driverButtonRS.onTrue(m_DriveTrainPID.WheelzLock());
    driverButtonB.onTrue(m_DriveTrainPID.ZeroGyro());
    driverButtonA.onTrue(m_DriveTrainPID.toggleFieldRelativeEnable());
    // WP - DO NOT UNCOMMENT WITHOUT TALKING TO WARD
    driverButtonOptions.onTrue(m_DriveTrainPID.resetPose2d());
  //  m_Arm.setDefaultCommand(new AutoRotateArmCommand(m_Arm));
    
  
    
    
    driverButtonY.whileTrue(m_Shooter.RunShooter());
    driverButtonY.whileFalse(m_Shooter.StopShooter());
    
    manipButtonB.whileTrue(m_Intake.RunIntakeWheels());
    manipButtonB.whileFalse(m_Intake.StopIntakeWheels());
    manipButtonLeft.whileTrue(m_Intake.RunPassthrough());
    manipButtonLeft.whileFalse(m_Intake.StopPassthrough());
    
   driverButtonLeft.whileTrue(m_Shooter.AngleDownShooter());//moves down
   driverButtonLeft.onFalse(m_Shooter.AngleStop());
   driverButtonRight.whileTrue(m_Shooter.AngleUpShooter()); //moves up
   driverButtonRight.onFalse(m_Shooter.AngleStop()); 
    ManippovUp.onTrue(new AutoIntakeUp(m_Intake));   
    ManippovDown.onTrue(new AutoIntakeDown(m_Intake));    
    //manipButtonLeft.onTrue(new AutoIntakeDown(m_Intake)); /*m_Intake.IntakePosLower()*/
    //manipButtonRight.onTrue(m_Intake.IntakePosRaise());
    //manipButtonLeft.onFalse(m_Intake.StopIntake());
    //manipButtonRight.onFalse(m_Intake.StopIntake());

    manipButtonX.whileTrue(m_Hanger.LeftHangUp());
    manipButtonY.whileTrue(m_Hanger.RightHangUp());
    driverButtonLeft.whileTrue(m_DriveTrainPID.Break());

  }

  private void configureShuffleboard() {
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Auto 1", 1);
    m_chooser.addOption("Auto 2", 2);
    m_chooser.addOption("Auto 3", 3);

    SmartDashboard.putData(m_chooser);

    // SmartDashboard.putData(m_DriveTrainPID.GetPose2d().getTranslation());

  }

  public Command getAutonomousCommand() {
    //loads New Auto auto file
       //return new PathPlannerAuto("New Auto");
      return new SequentialCommandGroup( 
        new InstantCommand( () -> m_DriveTrainPID.resetPose(new Pose2d(1.00, 5.00, new Rotation2d(0)))),
        new WaitCommand(3.0),
       autoChooser.getSelected()
      );

  }
}