package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class SwerveDriveCommand extends Command {
  DoubleSupplier m_leftY;
  DoubleSupplier m_leftX;
  DoubleSupplier m_rightX;
  DoubleSupplier m_AccelerateRT;
  DriveTrain m_DriveTrain;
  RobotContainer m_RobotContainer;

  private BooleanSupplier m_RunHalfSpeed;

  private static final double kDriveMaxSpeed = 0.6 * DriveTrain.kMaxSpeed;
  private static final double kTurnMaxSpeed = 0.6 * DriveTrain.kMaxTurnAngularSpeed;

  public SwerveDriveCommand(
      DoubleSupplier _leftY,
      DoubleSupplier _leftX,
      DoubleSupplier _rightX,
      DoubleSupplier _AccelerateRT,
      DriveTrain _dTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    m_AccelerateRT = _AccelerateRT;
    m_RunHalfSpeed = () -> false;
    addRequirements(m_DriveTrain);
  }

  public SwerveDriveCommand(
      DoubleSupplier _leftY,
      DoubleSupplier _leftX,
      DoubleSupplier _rightX,
      DoubleSupplier _AccelerateRT,
      DriveTrain _dTrain,
      BooleanSupplier _halfSpeedCondition) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    m_AccelerateRT = _AccelerateRT;
    m_RunHalfSpeed = _halfSpeedCondition;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RT, x, y, rot;
    double AccelerateRT = m_AccelerateRT.getAsDouble();
    double leftX = m_leftX.getAsDouble();
    double leftY = m_leftY.getAsDouble();
    double rightX = m_rightX.getAsDouble();

    // Finds the X Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    x = MathUtil.applyDeadband(-leftX, Constants.Controller.deadzone);

    // Finds the Y Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    y = MathUtil.applyDeadband(-leftY, Constants.Controller.deadzone);

    // Finds the X Value of the Right Stick on the Controller and Takes Care of
    // Joystick Drift
    rot = MathUtil.applyDeadband(-rightX, Constants.Controller.deadzone);

    RT = AccelerateRT;

    double normalizingFactor = Math.hypot(x, y);
    if (normalizingFactor > 0) {
      x /= normalizingFactor;
      y /= normalizingFactor;
    }
    double xSpeed = y * SwerveDriveCommand.kDriveMaxSpeed * RT;
    double ySpeed = x * SwerveDriveCommand.kDriveMaxSpeed * RT;
    double rotSpeed = rot * SwerveDriveCommand.kTurnMaxSpeed;

    if (m_RunHalfSpeed.getAsBoolean() == true) {
      xSpeed /= 2;
      ySpeed /= 2;
      rotSpeed /= 2;
    }

    SmartDashboard.putNumber("DriveTrain/Controller/Command/X Speed", xSpeed);
    SmartDashboard.putNumber("DriveTrain/Controller/Command/Y Speed", ySpeed);

    m_DriveTrain.drive(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
