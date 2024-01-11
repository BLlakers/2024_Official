
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

import frc.robot.subsystems.DriveTrainPID;
import frc.robot.RobotContainer;

public class SwerveDriveCommand extends Command {
  DoubleSupplier m_leftY;
  DoubleSupplier m_leftX;
  DoubleSupplier m_rightX;
  // double leftY;
  // double leftX;
  // double rightX;
  DriveTrainPID m_DriveTrain;
  RobotContainer m_RobotContainer;
  // double x;
  // double y;
  // double rot;
  // double w1ca;
  // double w2ca;
  // double w3ca;
  // double w4ca;

  public SwerveDriveCommand(DoubleSupplier _leftY, DoubleSupplier _leftX, DoubleSupplier _rightX,
      DriveTrainPID _dTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * m_DriveTrain.brDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.frDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.blDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.flDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double x;
    Double y;
    Double rot;
    Double leftX = m_leftX.getAsDouble();
    Double leftY = m_leftY.getAsDouble();
    Double rightX = m_rightX.getAsDouble();
    // System.out.println();

    // Finds the X Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(leftX) < Constants.deadzone) {
      x = 0.0;
    } else {
      x = -leftX;
    }

    // Finds the Y Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(leftY) < Constants.deadzone) {
      y = 0.0;
    } else {
      y = -leftY;
    }

    // Finds the X Value of the Right Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(rightX) < Constants.deadzone) {
      rot = 0.0;
    } else {
      rot = -rightX;
    }

    // Swerve drive uses a different Y and X than expected!

    m_DriveTrain.drive(y, x, rot, m_DriveTrain.FieldRelativeEnable, m_DriveTrain.WheelLock);
    m_DriveTrain.updateOdometry();
    Pose2d pose = m_DriveTrain.GetPose2d();
    System.out.println(pose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
