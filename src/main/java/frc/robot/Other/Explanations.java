package frc.robot.Other;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

/**
 * Here, Explanations of How everything works are held. 
 * <p> When Within The class:
 * <p> {@code - Hover over the Green Part to Get a basic explanation}
 * <p> {@code - Hover over the Yellow Part for a More Detailed explanation}
 * <p> {@code - Hover over The Blue Part to find what your getting an Explanation of}
 * <p> EX: int {@link #DriveTrainExplanation} (Blue) = {@link DriveTrain} (Green) {@link DriveTrain#Explanation} (Yellow)
 * <p> Sub Explanations are down Below:
 * <ul> <li> Init Sendable:
 * <ul> <li> Sendable objects are things that can be sent to the smart dashboard (WHEN TALKING ABOUT FRC!!!!).
 * <li> Initializing a sendable object sends that object to the smart dashboard.
 * <li> To send other things to the smart dashboard, Use the sendable builder (the thing that sends stuff)
 * <li> EX: {@link Hanger#initSendable} and {@link DriveTrain#initSendable}
 * <li> In order to send variables, you must do {@code builder.addDoubleProperty} (Instead of Double do variable type of choice)
 * </ul> 
 */
public class Explanations {


  public abstract class SubsystemExplanations {

    int DriveTrainExplanation = DriveTrain.Explanation();

    int HangerExplanation = Hanger.Explanation();

    int HangerModuleExplanation = HangerModule.Explanation();

    int IntakeExplanation = Intake.Explanation();

    int IntakeWheelsExplanation = IntakeWheels.Explanation();

    int LimelightExplanation = Limelight.Explanation();

    int ShooterExplanation = Shooter.Explanation();

    int SwerveModuleExplanation = SwerveModule.Explanation();
  }


  public abstract class RobotExplanations {

    int ConstantsExplanation = Constants.Explanation();

    int RobotExplanation = Robot.Explanation();

    int RobotContainerExplanation = RobotContainer.Explanation();

    // Main.java Is where we initialize our robot.
    // It is told which code based (Timed or Command Based or Custom) via the Robot.java class (As
    // seen in the Main File)
    // DO NOT CREATE ANYTHING IN MAIN.JAVA

  }


  public abstract class CommandExplanation {

    int AprilAlignToSpeakerRadiallyCommandExplanation = AprilAlignToSpeakerRadiallyCommand.Explanation();

    int AprilAlignToTransformCommandExplanation = AprilAlignToTransformCommand.Explanation();

    int AutoIntakeExplanation = AutoIntake.Explanation();

    int AutoShooterExplanation = AutoShooter.Explanation();

    int OrientShooterAngleExplanation = OrientShooterAngle.Explanation();

    int OrientShooterAngleByEncoderValueExplanation = OrientShooterAngleByEncoderValue.Explanation();

    int SwerveDriveCommandExplanation = SwerveDriveCommand.Explanation();
  }
}
