package frc.robot;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

public final class Constants {
    // Robot
    public static int PHChannel = 30; // REV Pneumatic Hub
    public static int PDHChannel = 20; // REV Power Distribution Hub
    public static double kWheelDiameterM = Inches.of(4).in(Meters); // 4 Inches
    public static double kWheelCircumference = Math.PI * kWheelDiameterM; //this is in meters
    // 1 meter = Inches 39.3701
    public static double NeoEncoderCountsPerRev = 42;
    public static double NeoRevPerEncoderCounts = 1/NeoEncoderCountsPerRev;
    //MISC
    public static double MetersToInches = Meters.of(1.0).in(Inches);
    public static double InchesToMeters = Inches.of(1.0).in(Meters);
    public static double MagEncoderCountsPerRev = 4096; 
    public static double MagRevPerEncoderCounts = 1/MagEncoderCountsPerRev;
    public static double GearRatio = 8.16;
    public static double driveEncoderConversion = GearRatio * kWheelCircumference;
    //ARM 
    public static int armMotorChannel1 = 9;
    public static int armMotorChannel2 = 10;
    public static double PositionDown = 0;
    public static double PositionPickup = 30; // 69

    public static double[] Positions = { PositionDown, PositionPickup };
    public static double ArmTolerance = 3;

    // Controller
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
    public static double deadzone = 0.1;

    // Drive Train
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
    public static double spinTolerance = 4.2;
    public static double length = 0.58;
    public static double width = 0.58;

}