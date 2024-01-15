package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AlignCommand extends Command {
  DriveTrain DriveTrain;
  DoubleSupplier angle;
/**
 * Creates a Constuctor for our AlignCommand
 * @param DriveTrain
 * @param angle
 */
  public AlignCommand(DriveTrain DriveTrain, DoubleSupplier angle) {
    this.DriveTrain = DriveTrain;
    this.angle = angle;
    addRequirements(DriveTrain);
  }

  @Override
  public void execute() { // Runs multiple times
    double move = 0.0;
    SmartDashboard.putNumber("command angle", angle.getAsDouble());
    // figuring out which way to drive
  if (angle.getAsDouble() >= 3) {
      // too far to right so it slowly moves to the left
      move = -0.1;
    } else if (angle.getAsDouble() <= -3) {
      // too far to the left so it slowly moves to the right
      move = 0.1;
  }
  if (angle.getAsDouble() == 9.6) {
      DriveTrain.drive(0, 0, 0, false, false);
    } else {
      DriveTrain.drive(0, move, 0, false, false);
    }
  }
  @Override
  public void end(boolean interrupted) {

  }
  public boolean isFinished() {
    return false;
  }
}