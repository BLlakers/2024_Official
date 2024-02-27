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
  public static class Drive {
    public final static Translation2d SMFrontRightLocation = new Translation2d(0.285, -0.285);
    public final static Translation2d SMFrontLeftLocation = new Translation2d(0.285, 0.285);
    public final static Translation2d SMBackLeftLocation = new Translation2d(-0.285, 0.285);
    public final static Translation2d SMBackRightLocation = new Translation2d(-0.285, -0.285);
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), // Translation constants
        new PIDConstants(3, 0, 0), // Rotation constants
        3.68, // what should be our robots fastest chassis speeds in m/s
        0.3875, // The radius of the robot in meters
        new ReplanningConfig());
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        0, 0, 0, new Rotation3d(0, 0, 0)); // we do conversion in limelight. would normally tell robot where the camera
                                           // is relative to the center of the bot.
    public static final TrapezoidProfile.Constraints kthetaController = new TrapezoidProfile.Constraints(
        DriveTrain.kMaxAngularSpeed, DriveTrain.kModuleMaxAngularAcceleration);
  }

  public static class Conversion {
    public final static double driveEncoderCtsperRev = 6.8;
    public final static double kWheelDiameterM = Inches.of(4).in(Meters);
    public final static double kWheelCircumference = Math.PI * kWheelDiameterM;
    public final static double NeoEncoderCountsPerRev = 42;
    public final static double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
    public final static double NeoMaxSpeedRPM = 5820;
    public final static double MagEncoderCountsPerRev = 4096;
    public final static double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
    public final static double DriveGearRatio = 8.14;
    public final static double TurnGearRatio = 12.8;
    public final static double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
  }

  public static class Controller {
    public final static int DriverControllerChannel = 0;
    public final static int ManipControllerChannel = 1;
    public final static int buttonA = 1;
    public final static int buttonB = 2;
    public final static int buttonX = 3;
    public final static int buttonY = 4;
    public final static int buttonLeft = 5;
    public final static int buttonRight = 6;
    public final static int buttonOptions = 7;
    public final static int buttonStart = 8;
    public final static int buttonLS = 9;
    public final static int buttonRS = 10;
    public final static double deadzone = 0.17;
    public final static double RTdeadzone = .01;
  }

  public static class Port {
    public final static int blSteerMtrC = 1;
    public final static int blDriveMtrC = 2;
    public final static int flDriveMtrC = 3;
    public final static int flSteerMtrC = 4;
    public final static int frSteerMtrC = 5;
    public final static int frDriveMtrC = 6;
    public final static int brDriveMtrC = 7;
    public final static int brSteerMtrC = 8;
    public final static int blTurnEncoderC = 0;
    public final static int flTurnEncoderC = 1;
    public final static int frTurnEncoderC = 2;
    public final static int brTurnEncoderC = 3;
    public final static int PHChannel = 30; // REV Pneumatic Hub
    public final static int PDHChannel = 20; // REV Power Distribution Hub
  }

  public static class Intake {
    public final static int AngleMtrC = 15;
    public final static int WheelMtrC = 14;
   
  }

  // SHOOTER
  public static class Shooter {
    public final static int LeftMtrC = 11;
    public final static int RightMtrC = 12;
    public final static int AngleMtrC = 13;
    public final static int LimitSwitchTopDIO = -1; // TODO: add the digital input channel for this limit switch
    public final static int LimitSwitchBottomDIO = -1; // TODO: add the digital input channel for this limit
  }

  // HANGER
  public static class Hanger {
    public final static int LeftMtrC = 9;
    public final static int RightMtrC = 10;
  }

  public abstract class RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0;
    public static final double frTurnEncoderOffset = 0;
    public static final double blTurnEncoderOffset = 0;
    public static final double brTurnEncoderOffset = 0;
  }

  public class RobotVersion2023 extends RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0.3359;
    public static final double frTurnEncoderOffset = 0.730;
    public static final double blTurnEncoderOffset = .1819;
    public static final double brTurnEncoderOffset = 0.9262;
  }

  public class RobotVersion2024 extends RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0.6168;
    public static final double frTurnEncoderOffset = 0.777;
    public static final double blTurnEncoderOffset = 0.519;
    public static final double brTurnEncoderOffset = 0.625;
  }

  public static final RobotVersion defaultRobotVersion = RobotVersion.v2024;

}