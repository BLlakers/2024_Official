// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveAndDriveConstants;
/**
 * This is where our Swerve Functions and electrical objects pertaining to the DriveTrain are created. <br>
 * <br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {

    // kWheelCircumference used to be
   public final CANSparkMax driveMotor;
   public final CANSparkMax turningMotor;
   private final SparkPIDController drivePID;
   public final RelativeEncoder driveEncoder;
   private final DutyCycleEncoder turningEncoder;
   private final DigitalInput TurnEncoderInput;
   public final DutyCycle TurnPWMEncoder;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel     CAN ID for the drive motor.
     * @param turningMotorChannel   CAN ID for the turning motor.
     * @param driveEncoder          DIO input for the drive encoder channel A
     * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
     * @param turnOffset            offset from 0 to 1 for the home position of the
     *                              encoder
     */
     public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turnEncoderPWMChannel, double turnOffset) {
        driveMotor = new CANSparkMax(driveMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        driveMotor.setOpenLoopRampRate(0.1);
        drivePID = driveMotor.getPIDController();
        drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        drivePID.setSmartMotionMaxAccel(0.2, 0);
        drivePID.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(SwerveAndDriveConstants.rpstoPositionScaler);
        driveEncoder.setVelocityConversionFactor(SwerveAndDriveConstants.rpmToVelocityScaler);
        SwerveAndDriveConstants.turnEncoderPWMChannel = turnEncoderPWMChannel;
        TurnEncoderInput = new DigitalInput(turnEncoderPWMChannel);
        TurnPWMEncoder = new DutyCycle(TurnEncoderInput);
        SwerveAndDriveConstants.turnEncoderOffset = turnOffset;
        turningEncoder = new DutyCycleEncoder(TurnPWMEncoder);

        // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
        SwerveAndDriveConstants.m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        //SwerveAndDriveConstants.encoderBias = driveEncoder.getPosition(); //Whats the encoder Bias? //TODO

        driveEncoder.setPosition(0);
        turningEncoder.reset();
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(getTurnEncoderRadians()));
      }
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // the getVelocity() function normally returns RPM but is scaled in the
        // SwerveModule constructor to return actual wheel speed
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderRadians()));
    } 
/**
 * Gets a distance of where a module wants to get to. 
 * @return a SwerveModuleState
 */
    public SwerveModuleState getDifferentState() {
        return new SwerveModuleState((driveEncoder.getPosition() - SwerveAndDriveConstants.encoderBias) * SwerveAndDriveConstants.rpmToVelocityScaler * 60,
                new Rotation2d(getTurnEncoderRadians()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoderRadians()));

        // Calculate the drive output from the drive PID controller.

        final double signedAngleDifference = closestAngleCalculator(getTurnEncoderRadians(), state.angle.getRadians());
        double rotateMotorPercentPower = signedAngleDifference / (2 * Math.PI); // proportion error control //2

        driveMotor.set((state.speedMetersPerSecond / SwerveAndDriveConstants.kMaxSpeed) * Math.cos(rotateMotorPercentPower));
        turningMotor.set(1.6 * rotateMotorPercentPower);
    }

    /**
     * Applies the absolute encoder offset value and converts range
     * from 0-1 to 0-2 pi radians
     * 
     * @return Angle of the absolute encoder in radians
     */
    private double getTurnEncoderRadians() {
        double appliedOffset = (TurnPWMEncoder.getOutput() - SwerveAndDriveConstants.turnEncoderOffset) % 1;
        SmartDashboard.putNumber("PWMChannel " + Integer.toString(SwerveAndDriveConstants.turnEncoderPWMChannel), TurnPWMEncoder.getOutput());
        return appliedOffset * 2 * Math.PI;
    }

    /**
     * Calculates the closest angle and direction between two points on a circle.
     * 
     * @param currentAngle
     *                     <ul>
     *                     <li>where you currently are
     *                     </ul>
     *                     </li>
     * @param desiredAngle
     *                     <ul>
     *                     <li>where you want to end up
     *                     </ul>
     *                     </li>
     * @return
     *         <ul>
     *         <li>signed double of the angle (rad) between the two points
     *         </ul>
     *         </li>
     */
    public double closestAngleCalculator(double currentAngle, double desiredAngle) {
        double signedDiff = 0.0;
        double rawDiff = currentAngle > desiredAngle ? currentAngle - desiredAngle : desiredAngle - currentAngle; // find the positive raw distance between the angles
        double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle
        if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
            signedDiff = ((2 * Math.PI) - modDiff); // full circle minus the angle
            if (desiredAngle > currentAngle)
                signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
        } else {
            signedDiff = modDiff;
            if (currentAngle > desiredAngle)
                signedDiff = signedDiff * -1;
        }
        return signedDiff;
    }
    /**
     * Tells our motors to stop
     */
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }










// From Arm.java TODO 





}
