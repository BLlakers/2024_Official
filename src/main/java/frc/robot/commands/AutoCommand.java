package frc.robot.commands;

// import java.io.Serial;
import java.util.function.DoubleSupplier;
// import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainPID;

public class AutoCommand extends Command {
  DriveTrainPID m_DriveTrain; // Creates an object DriveTrain
  double leftY; // Creates a Variable for the Left joystick Y position (fake controller)
  double leftX; // Creates a Variable for the Left joystick X position (fake controller)
  double rightX; // Creates a Variable for the right joystick X position (fake controller)
  double counter; // Creates a Variable that counts the amount of time we keep the shooter on
  int m_AutoMode; // If AutoMode = 1 then run routine 1, if AutoMode = to 2 then run 2, if
                  // AutoMode equal to 3 run routine 3, otherwise don't run.

  public AutoCommand(DriveTrainPID m_DriveTrain, int m_AutoMode) { // Creates a contrusctor for auto command (How things
                                                                   // get
    // set up)
    this.m_DriveTrain = m_DriveTrain;
    this.m_AutoMode = m_AutoMode;
    addRequirements(this.m_DriveTrain);
  }

  @Override
  public void initialize() { // Runs once at the beginning of the command
    m_DriveTrain.m_backRight.m_driveMotor.getEncoder().setPosition(0); // Sets the position of the encoder
    m_DriveTrain.m_frontRight.m_driveMotor.getEncoder().setPosition(0);
    m_DriveTrain.m_backLeft.m_driveMotor.getEncoder().setPosition(0);
    m_DriveTrain.m_frontLeft.m_driveMotor.getEncoder().setPosition(0);
    counter = 0; // Sets counter = 0

    m_DriveTrain.drive(0, 0, 0, false, false);

  }

  @Override
  public void execute() { // Runs multiple times

    leftY = -0.20; // Tells controller to move backwards on the Y axis, was -0.35
    leftX = 0.0; // Tells controller not to move
    rightX = 0.0; // Tells controller not to move (No RightY because it doesn't do anything)
    /*
     * double w1ca = (-1 * m_DriveTrain.getPosition(m_DriveTrain.frEncoder.get(),
     * 267.4)) + 360;
     * double w2ca = (-1 * m_DriveTrain.getPosition(m_DriveTrain.flEncoder.get(),
     * 120.7)) + 360;
     * double w3ca = (-1 * m_DriveTrain.getPosition(m_DriveTrain.blEncoder.get(),
     * 64.7)) + 360;
     * double w4ca = (-1 * m_DriveTrain.getPosition(m_DriveTrain.brEncoder.get(),
     * 335.2)) + 360;
     */
    // Author: Jared
    // Date: 1/25/2023
    // The purpose of the following set of instructions is to re-center the wheels
    // to face forward, based on the sensors coming from the swerve drive.
    // If The front right wheel is > 7 degress and < 173 OR > 187 and < 353 it will
    // tell the robot to not move foward and it will re center the wheels

    /*
     * if ((w1ca > 7 && w1ca < 173) || (w1ca > 187 && w1ca < 353)) { // If The front
     * right wheel is > 7 degress and < 173
     * // OR > 187 and < 353 it will tell the robot to not
     * // move foward and it will re center the wheels
     * m_DriveTrain.drive(0, 0, 0, false, false); // says not to drive
     * } else if ((w2ca > 7 && w2ca < 173) || (w2ca > 187 && w2ca < 353)) { // If
     * The front left wheel is > 7 degress and
     * // < 173 OR > 187 and < 353 it will tell the
     * // robot to not move foward and it will re
     * // center the wheels
     * m_DriveTrain.drive(0, 0, 0, false, false); // says not to drive
     * } else if ((w3ca > 7 && w3ca < 173) || (w3ca > 187 && w3ca < 353)) { // If
     * The back left wheel is > 7 degress and
     * // < 173 OR > 187 and < 353 it will tell the
     * // robot to not move foward and it will re
     * // center the wheels
     * m_DriveTrain.drive(0, 0, 0, false, false); // says not to drive
     * } else if ((w4ca > 7 && w4ca < 173) || (w4ca > 187 && w4ca < 353)) { // If
     * The back right wheel is > 7 degress and
     * // < 173 OR > 187 and < 353 it will tell the
     * // robot to not move foward and it will re
     * // center the wheels
     * m_DriveTrain.drive(0, 0, 0, false, false); // says not to drive
     * } else {
     */
    Pose2d currentPose = m_DriveTrain.GetPose2d();
    

    if (m_AutoMode == 1) {
      if (Math.abs(m_DriveTrain.m_backRight.m_driveMotor.getEncoder().getPosition()) < 115.0597) { // Drives until the
                                                                                                   // encoder is at
        // rotations on the motor. 1 motor
        // rotation = 8.14 wheel rotation
        // 115.0597
        m_DriveTrain.drive(-leftY, leftX, rightX, false, false);
      } else {
        m_DriveTrain.drive(0, 0, 0, false, false);
        counter = counter + 1;
      }
    }

    else if (m_AutoMode == 2) {
      if (Math.abs(m_DriveTrain.m_backRight.m_driveMotor.getEncoder().getPosition()) < 50) { // Drives until the
                                                                                               // encoder is at
        // rotations on the motor. 1 motor
        // rotation = 8.14 wheel rotation (11
        // inches = wheel)
        currentPose = m_DriveTrain.GetPose2d();
        System.out.print(currentPose);
        m_DriveTrain.drive(-leftY, leftX, rightX, false, false);
        currentPose = m_DriveTrain.GetPose2d();
        System.out.print(currentPose);
        
      } else {
        m_DriveTrain.drive(0, 0, 0, false, true);
        counter = counter + 1;
      }
    }

    else if (m_AutoMode == 3) {
      if (Math.abs(m_DriveTrain.m_backRight.m_driveMotor.getEncoder().getPosition()) < 125.0597) { // Drives until the
                                                                                                   // encoder is at
        // rotations on the motor. 1 motor
        // rotation = 8.14 wheel rotation
        m_DriveTrain.drive(-leftY, leftX, rightX, false, false);
      } else {
        m_DriveTrain.drive(0, 0, 0, false, false);
        counter = counter + 1;
      }
    }
    // }
  }

  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.drive(0, 0, 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
