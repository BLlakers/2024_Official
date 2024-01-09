package frc.robot;

public final class Constants {
    // Robot
    public static int PHChannel = 30; // REV Pneumatic Hub
    public static int PDHChannel = 20; // REV Power Distribution Hub

    // Arm
    // 1 meter = 39.3701
    public static double MetersToInches = 39.3701;
    public static int armMotorChannel1 = 9;
    public static int armMotorChannel2 = 10;
    // public static int armMotor1 = 9;
    // public static int armMotor2 = 10;
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