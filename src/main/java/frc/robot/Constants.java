package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Other.RobotVersion;
import frc.robot.subsystems.DriveTrain;

public final class Constants {
public class Drive{
  public final static Translation2d SMFrontRightLocation = new Translation2d(0.285, -0.285);
  public final static Translation2d SMFrontLeftLocation = new Translation2d(0.285, 0.285);
  public final static Translation2d SMBackLeftLocation = new Translation2d(-0.285, 0.285);
  public final static Translation2d SMBackRightLocation = new Translation2d(-0.285, -0.285);
  public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
  new PIDConstants(5, 0, 0), // Translation constants 
  new PIDConstants(3, 0, 0), // Rotation constants 
  3.68, //what should be our robots fastest chassis speeds in m/s
  0.3875, //The radius of the robot in meters
  new ReplanningConfig());
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
      0,0,0, new Rotation3d(0,0,0)); // we do conversion in limelight. would normally tell robot where the camera is relative to the center of the bot.
    public static final TrapezoidProfile.Constraints kthetaController = new TrapezoidProfile.Constraints(DriveTrain.kMaxAngularSpeed,DriveTrain.kModuleMaxAngularAcceleration);
}
public class Conversion{
    public static double driveEncoderCtsperRev = 6.8;
    public static double kWheelDiameterM = Inches.of(4).in(Meters);
    public static double kWheelCircumference = Math.PI * kWheelDiameterM;
    public static double NeoEncoderCountsPerRev = 42;
    public static double NeoRevPerEncoderCounts = 1/NeoEncoderCountsPerRev;
    public static double NeoMaxSpeedRPM = 5820;
    public static double MagEncoderCountsPerRev = 4096; 
    public static double MagRevPerEncoderCounts = 1/MagEncoderCountsPerRev;
    public static double DriveGearRatio = 8.14;
    public static double TurnGearRatio = 12.8;
    public static double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
}
    public class Controller{
    public static int DriverControllerChannel = 0;
    public static int ManipControllerChannel = 1;
    public static int buttonA = 1;
    public static int buttonB = 2;
    public static int buttonX = 3;
    public static int buttonY = 4;
    public static int buttonLeft = 5;
    public static int buttonRight = 6;
    public static int buttonOptions = 7;
    public static int buttonLS = 9;
    public static int buttonRS = 10;
    public static double deadzone = 0.17;
    public static double RTdeadzone = .01;
    }
    public class Port{
    public static int blSteerMotorChannel = 1;
    public static int blDriveMotorChannel = 2;
    public static int flDriveMotorChannel = 3;
    public static int flSteerMotorChannel = 4;
    public static int frSteerMotorChannel = 5;
    public static int frDriveMotorChannel = 6;
    public static int brDriveMotorChannel = 7;
    public static int brSteerMotorChannel = 8;
    public static int blEncoderChannel = 0;
    public static int flEncoderChannel = 1;
    public static int frEncoderChannel = 2;
    public static int brEncoderChannel = 3;
    public static int PHChannel = 30; // REV Pneumatic Hub
    public static int PDHChannel = 20; // REV Power Distribution Hub
    }

    public abstract class RobotVersionConstants {
      public static final double flTurnEncoderOffset = 0;
      public static final double frTurnEncoderOffset = 0;
      public static final double blTurnEncoderOffset = 0;
      public static final double brTurnEncoderOffset = 0;    
    }

    public class RobotVersion2023 extends RobotVersionConstants
    {
      public static final double flTurnEncoderOffset = 0.3359; // , 0.730
      public static final double frTurnEncoderOffset = 0.730;// 0.3359);
      public static final double blTurnEncoderOffset = .1819;// 1.1819);
      public static final double brTurnEncoderOffset = 0.9262;// , 0.9262
    } 

     public class RobotVersion2024 extends RobotVersionConstants
    {
      public static final double flTurnEncoderOffset = 0.6168;
      public static final double frTurnEncoderOffset = 0.777;
      public static final double blTurnEncoderOffset = 0.519;
      public static final double brTurnEncoderOffset = 0.625;
    } 

    public static final RobotVersion defaultRobotVersion = RobotVersion.v2024;


}