package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
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
        DriveTrain.kMaxSpeed, // what should be our robots fastest chassis speeds in m/s
        0.3875, // The radius of the robot in meters
        new ReplanningConfig(true,false));
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        0, 0, 0, new Rotation3d(0, 0, 0)); // we do conversion in limelight. would normally tell robot where the camera
                                           // is relative to the center of the bot.
    public static final TrapezoidProfile.Constraints kthetaController = new TrapezoidProfile.Constraints(
        DriveTrain.kMaxAngularSpeed, DriveTrain.kModuleMaxAngularAcceleration);
  }

  public static class Conversion {
    public static final double driveEncoderCtsperRev = 6.8;
    public static final double kWheelDiameterM = Inches.of(4).in(Meters);
    public static final double kWheelCircumference = Math.PI * kWheelDiameterM;
    public static final double NeoEncoderCountsPerRev = 42;
    public static final double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
    public static final double NeoMaxSpeedRPM = 5820;
    public static final double MagEncoderCountsPerRev = 4096;
    public static final double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
    public static final double DriveGearRatio = 8.14;
    public static final double TurnGearRatio = 12.8;
    public static final double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
  }

  public static class Controller {
    public static final int DriverControllerChannel = 0;
    public static final int ManipControllerChannel = 1;
    public static final int DebugControllerChannel = 2;
    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
    public static final int buttonLeft = 5;
    public static final int buttonRight = 6;
    public static final int buttonOptions = 7;
    public static final int buttonStart = 8;
    public static final int buttonLS = 9;
    public static final int buttonRS = 10;
    public static final double deadzone = 0.17;
    public static final double RTdeadzone = .01;
  }

  public static class AprilTagID {
    public static final int PracticeSpeakerCenter = 1;
    public static final int BlueSpeakerCenter = 7;
    public static final int RedSpeakerCenter = 4;

    public static final int BlueStageCenter = 14;
    public static final int RedStageCenter = 13;

    public static final int BlueStageLeft = 15;
    public static final int RedStageRight = 12;

    public static final int RedStageLeft = 11;
    public static final int BlueStageRight = 16;

    public static final Pose2d BlueSpeakerCenterPose = new Pose2d(); // TODO
    public static final Pose2d RedSpeakerCenterPose = new Pose2d(); // TODO
  }

  public static class Port {
    public static final int blSteerMtrC = 1;
    public static final int blDriveMtrC = 2;
    public static final int flDriveMtrC = 3;
    public static final int flSteerMtrC = 4;
    public static final int frSteerMtrC = 5;
    public static final int frDriveMtrC = 6;
    public static final int brDriveMtrC = 7;
    public static final int brSteerMtrC = 8;
    public static final int blTurnEncoderDIOC = 0;
    public static final int flTurnEncoderDIOC = 1;
    public static final int frTurnEncoderDIOC = 2;
    public static final int brTurnEncoderDIOC = 3;
    public static final int hangerLeftMagSwitchDIOC = 7;
    public static final int hangerRightMagSwitchDIOC = 8;
    public static final int PHChannel = 30; // REV Pneumatic Hub
    public static final int PDHChannel = 20; // REV Power Distribution Hub
  }

  public static class Intake {
    public static final int AngleMtrC = 15;
    public static final int WheelMtrC = 14;
  }

  // SHOOTER
  public static class Shooter {
    public static final int LeftMtrC = 11;
    public static final int RightMtrC = 12;
    public static final int AngleMtrC = 13;
    public static final int LimitSwitchTopDIO = 4;
    public static final int LimitSwitchBottomDIO =
        -1; // TODO: add the digital input channel for this limit
  }

  // HANGER
  public static class Hanger {
    public static final int LeftMtrC = 9;
    public static final int RightMtrC = 10;
  }

  public abstract class RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0;
    public static final double frTurnEncoderOffset = 0;
    public static final double blTurnEncoderOffset = 0;
    public static final double brTurnEncoderOffset = 0;
  }

  public class RobotVersion2023 extends RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0.3459;
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
