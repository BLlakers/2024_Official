package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainPID;

public class AlignCommand extends Command {
  DriveTrainPID m_DriveTrain; // Creates an object DriveTrain
  DoubleSupplier m_angle;

  public AlignCommand(DriveTrainPID _DriveTrain, DoubleSupplier _angle) { // Creates a contrusctor for auto command (How
    // things get set up)
    m_DriveTrain = _DriveTrain;
    m_angle = _angle;
    addRequirements(m_DriveTrain);
  }

  @Override
  public void initialize() { // Runs once at the beginning of the command

  }

  @Override
  public void execute() { // Runs multiple times
    double move = 0.0;

    // this is old stuff, but i am keeping it just as a referance and just incase it
    // is needed again
    // turn2 must somehow become a doubble supplier and the drivetrain is off too
    // finding the turning amount needed
    Double turn1 = 0.035; // how much the motor must turn to turn one degree, this number IS WRONG: it
                          // should be either 0.03555 or, i bet it is actully 0.02261
    Double turn2 = turn1 * m_angle.getAsDouble();
    // Double turn = (turn2.getAsDoubleSupplier());
    SmartDashboard.putNumber("command angle", m_angle.getAsDouble());
    // end of old stuff

    // figuring out which way to drive
    if (m_angle.getAsDouble() >= 3) {
      // too far to right so it slowly moves to the left
      move = -0.1;
    }

    else if (m_angle.getAsDouble() <= -3) {
      // too far to the left so it slowly moves to the right
      move = 0.1;
    }

    // fianlly drivinG
    Double move1 = move;
    // added to fix error in the last statemnt because it said move was "not final"

    if (m_angle.getAsDouble() == 9.6) {
      m_DriveTrain.drive(0, 0, 0, false, false);
    } else {

      m_DriveTrain.drive(0, move, 0, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {

  }
}