package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.SwerveModule;
/**
 * This is where all variables created outside of functions or Constructors are held.
 */
public final class Constants {
    
    // Autonomous stuff for Monday, the 15th of january
    
     public static final HolonomicPathFollowerConfig pathFollowerConfig = new
      HolonomicPathFollowerConfig(
      new PIDConstants(1, 0, 0), // Translation constants
      new PIDConstants(1, 0, 0), // Rotation constants
      SwerveAndDriveConstants.kChassisMaxSpeed,
      SwerveAndDriveConstants.frontLeftLocation.getNorm(), // Drive base radius (distance from center to furthest module)
      new ReplanningConfig());

    public class MiscConstants {
        public static double deadzone = 0.1;
        public static int ArmPosition = 1;
    }

    public class SwerveAndDriveConstants {
        public final static double kSwerveMaxAcceleration = 2 * Math.PI;
        public final static double kSwerveMaxSpeed = 1;
        public final static double kChassisMaxSpeed = Math.PI / 3; // 1/2 rotation per second
        public final static double kChassisMaxAcceleration = Math.PI / 3;
        public final static TrapezoidProfile.Constraints kSwervethetaConstraints = new TrapezoidProfile.Constraints(kSwerveMaxSpeed, kSwerveMaxAcceleration);
        public final static TrapezoidProfile.Constraints kChassisthetaContraints = new TrapezoidProfile.Constraints(kChassisMaxSpeed, kChassisMaxAcceleration);
        public static boolean WheelLock = false;
        public static boolean FieldRelativeEnable = true;
        public static double turnEncoderOffset;
        public static double encoderBias = 0; // encoder stuff for rotation
        public static int turnEncoderPWMChannel;
        public final static Translation2d frontRightLocation = new Translation2d(0.285, -0.285);
        public final static Translation2d frontLeftLocation = new Translation2d(0.285, 0.285);
        public final static Translation2d backLeftLocation = new Translation2d(-0.285, 0.285);
        public final static Translation2d backRightLocation = new Translation2d(-0.285, -0.285);
        public final static double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared 
         // WP this seemed to work don't know why // 3.68 meters per second or 12.1 ft/s (max speed of SDS Mk3 with Neo motor)
       
        public final static double rpstoPositionScaler = (RobotConstants.kWheelCircumference * ConversionConstants.driveEncoderCtsperRev) / (2 * Math.PI);// First thought for ratio = (Constants.kWheelDiameterM * Constants.NeoEncoderCountsPerRev) / (Constants.GearRatio * (Math.PI * 2));
        public final static double rpmToVelocityScaler = 3 * (RobotConstants.kWheelCircumference / RobotConstants.GearRatioMK3)/ 60; // SDS Mk3 standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
    }
    
    //public static final double kModuleMaxAngularAcceleration = Math.PI / 3; where do we get this value from???????????????, Would be in Swerve Constants
    

    public class RobotConstants {
        public static double kWheelDiameterM = Inches.of(4).in(Meters); // Our wheel diameter, which is 4 Inches
        public static double kWheelCircumference = Math.PI * kWheelDiameterM; // Our wheel Circumfrence, which is Pi * our wheel diameter
        public static double GearRatioMK3 = 8.16; // Gear 1 gear ratio for a MK3 module (or the slowest version)
        public static double GearRatioMK4 = 8.14; // Gear 1 gear ratio for a MK4 module (or the slowest version)
        public static double spinTolerance = 4.2; //Unused Variable 
        public static double length = 0.58;
        public static double width = 0.58;
    }


    public class ButtonConstants {
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
    }


    public class ChannelConstants {
        // Devices channels
        public static int PHChannel = 30; // REV Pneumatic Hub
        public static int PDHChannel = 20; // REV Power Distribution Hub

        // Motor Channels
        public static int blSteerMotorChannel = 1;
        public static int blDriveMotorChannel = 2;
        public static int flDriveMotorChannel = 3;
        public static int flSteerMotorChannel = 4;
        public static int frSteerMotorChannel = 5;
        public static int frDriveMotorChannel = 6;
        public static int brDriveMotorChannel = 7;
        public static int brSteerMotorChannel = 8;
        public static int MotorChannel9 = 9; // TODO name Changed when we add arm/ other motors
        public static int MotorChannel10 = 10;
        public static int MotorChannel11 = 11;
        public static int MotorChannel12 = 12;
        public static int MotorChannel13 = 13;
        public static int MotorChannel14 = 14;
        public static int MotorChannel15 = 15;

        // Encoders
        public static int blEncoderChannel = 0;
        public static int flEncoderChannel = 1;
        public static int frEncoderChannel = 2;
        public static int brEncoderChannel = 3;
    }
    public class ConversionConstants{
        public static double driveEncoderCtsperRev = 6.8;
        public static double NeoEncoderCountsPerRev = 42;
        public static double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
        public static double MetersToInches = Meters.of(1.0).in(Inches);
        public static double InchesToMeters = Inches.of(1.0).in(Meters);
        public static double MagEncoderCountsPerRev = 4096;
        public static double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
        public static double driveEncoderConversion = RobotConstants.GearRatioMK3 * RobotConstants.kWheelCircumference;
    }
}