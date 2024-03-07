package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class IntakePIDcommand extends ProfiledPIDCommand {
  enum DrivingStateEndCondition {
    PositionStopUp,
    PositionStopDown,
    PositionHold
  }

  private Intake m_Intake;
  private DrivingStateEndCondition m_endCondition;

  public IntakePIDcommand(Intake intake, Rotation2d goal, DrivingStateEndCondition endCondition) {
    super(
        new ProfiledPIDController(
            1.2,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.radiansToDegrees(50), Units.degreesToRadians(40))),
        () -> {
          return intake.GetIntakeMotorAngle().getRadians();
        },
        goal.getRadians(),
        (setpoint, output) -> intake.MoveIntake(setpoint),
        intake);
    m_Intake = intake;
    m_endCondition = endCondition;
    getController()
        .enableContinuousInput(
            -Math.PI, Math.PI); // finds the closest point on a circle and tells it to
    // go that way.
    getController().setTolerance(Units.degreesToRadians(5));
    addRequirements(m_Intake);
  }

  public static IntakePIDcommand IntakeUp(Intake intake) {
    return new IntakePIDcommand(intake, Intake.PosUpAngle, DrivingStateEndCondition.PositionStopUp);
  }

  public static IntakePIDcommand IntakeAmp(Intake intake) {
    return new IntakePIDcommand(intake, Intake.PosAmpAngle, DrivingStateEndCondition.PositionHold);
  }

  public static IntakePIDcommand IntakeDown(Intake intake) {
    return new IntakePIDcommand(
        intake, Intake.PosDownAngle, DrivingStateEndCondition.PositionStopDown);
  }

  @Override
  public void initialize() {
    super.initialize();
    getController().setTolerance(Units.degreesToRadians(0.5)); // set 0.5 degree tolerance for error
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_Intake.StopIntake();
  }

  @Override
  public boolean isFinished() {
    boolean endConditionMet = false;
    switch (m_endCondition) {
      case PositionHold:
        endConditionMet = false;
        break;

      case PositionStopUp:
        endConditionMet = this.IsIntakeUp();
        break;

      case PositionStopDown:
        endConditionMet = this.IsIntakeDown();
        break;
    }

    return getController().atGoal() || endConditionMet;
  }

  private boolean IsIntakeUp() {
    return m_Intake.GetIntakeMotorAngle().getDegrees() <= Intake.DrivingPositionUp.getDegrees();
  }

  private boolean IsIntakeDown() {
    return m_Intake.GetIntakeMotorAngle().getDegrees() >= Intake.DrivingPositionDown.getDegrees();
  }
}
