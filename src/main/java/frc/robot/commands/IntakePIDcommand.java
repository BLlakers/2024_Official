package frc.robot.commands;

import frc.robot.subsystems.*;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class IntakePIDcommand extends ProfiledPIDCommand {
  Intake m_Intake;

  public IntakePIDcommand(Intake intake, Rotation2d goal) {
    super(
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1, 1)),
        () -> {
          return intake.GetIntakeMotorAngle().getRadians();
        },
        goal.getRadians(),
        (setpoint, output) -> intake.MoveIntake(setpoint),
        intake);
    m_Intake = intake;
    getController()
        .enableContinuousInput(
            -Math.PI, Math.PI); // finds the closest point on a circle and tells it to go that way.
  }

  public static IntakePIDcommand IntakeUp(Intake intake) {
    return new IntakePIDcommand(intake, Intake.PosUpAngle);
  }

  public static IntakePIDcommand IntakeAmp(Intake intake) {
    return new IntakePIDcommand(intake, Intake.PosAmpAngle);
  }

  public static IntakePIDcommand IntakeDown(Intake intake) {
    return new IntakePIDcommand(intake, Intake.PosDownAngle);
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
    ;
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
