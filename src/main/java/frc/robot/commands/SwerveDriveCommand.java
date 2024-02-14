
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

import frc.robot.subsystems.DriveTrainPID;
import frc.robot.RobotContainer;

public class SwerveDriveCommand extends Command {
  DoubleSupplier m_leftY;
  DoubleSupplier m_leftX;
  DoubleSupplier m_rightX;
  DoubleSupplier m_AccelerateRT;
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

  public SwerveDriveCommand(DoubleSupplier _leftY, DoubleSupplier _leftX, DoubleSupplier _rightX, DoubleSupplier _AccelerateRT,
      DriveTrainPID _dTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    m_AccelerateRT = _AccelerateRT;
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
    Double RT;
    Double x;
    Double y;
    Double rot;
    double AccelerateRT = m_AccelerateRT.getAsDouble();
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
      rot = -Math.signum(rightX) * (Math.abs(rightX) - Constants.deadzone) / (1 - Constants.deadzone);
    }
    RT = AccelerateRT;

    double normalizingFactor = Math.sqrt(x*x + y*y);
    if (normalizingFactor > 0)
    {
      x /= normalizingFactor;
      y /= normalizingFactor;
    }
    // Swerve drive uses a different Y and X than expected!
    double xSpeed = y * DriveTrainPID.kMaxSpeed * RT;
    double ySpeed = x * DriveTrainPID.kMaxSpeed * RT;
    double rotSpeed = rot * DriveTrainPID.kMaxTurnAngularSpeed;
    SmartDashboard.putNumber("Robot/Controller/Command/X Speed", xSpeed);
    SmartDashboard.putNumber("Robot/Controller/Command/Y Speed", ySpeed);

    m_DriveTrain.drive(xSpeed, ySpeed, rotSpeed);
    Pose2d pose = m_DriveTrain.getPose2d();
    //System.out.println(pose);
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
