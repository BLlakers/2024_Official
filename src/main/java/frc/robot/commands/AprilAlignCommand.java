// this is semi right, this is set to align to the cameras center, not the
// robots center
/*
 * package frc.robot.commands;
 * import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 * import java.util.function.DoubleSupplier;
 * import edu.wpi.first.wpilibj2.command.Command;
 * import frc.robot.subsystems.DriveTrain;
 * 
 * public class AprilAlignCommand extends Command {
 * DriveTrain m_DriveTrain; // Creates an object DriveTrain
 * DoubleSupplier m_tx2;
 * 
 * public AprilAlignCommand(DriveTrain _DriveTrain, DoubleSupplier _tx2) { //
 * Creates a contrusctor for auto command (How things get set up)
 * m_DriveTrain = _DriveTrain;
 * m_tx2 = _tx2;
 * addRequirements(m_DriveTrain);
 * }
 * 
 * @Override
 * public void initialize() { // Runs once at the beginning of the command
 * 
 * }
 * 
 * @Override
 * public void execute() { // Runs multiple times
 * 
 * //turn2 must somehow become a doubble supplier and the drivetrain is off too
 * //finding the turning amount needed
 * Double turn1 = 0.035; //how much the motor must turn to turn one degree, this
 * number IS WRONG: it should be either 0.03555 or, i bet it is actully 0.02261
 * Double turn2 = turn1 * m_tx2.getAsDouble();
 * //Double turn = (turn2.getAsDoubleSupplier());
 * SmartDashboard.putNumber("command angle april tags", m_tx2.getAsDouble());
 * 
 * //movement for the left(left joystick thing) y on the "joystick"
 * DoubleSupplier lefty;
 * lefty = () -> 0.0;
 * 
 * //movement fo the left(left joystick thing) x on the "joystick"
 * DoubleSupplier leftx;
 * leftx = () -> 0.0;
 * 
 * //finaly driving
 * m_DriveTrain.drive(() -> 0, () -> 0, () -> 0);
 * m_DriveTrain.drive(lefty, leftx, () -> turn2);
 * //make this so it ONLY runs once
 * 
 * 
 * }
 * 
 * @Override
 * public void end(boolean interrupted) {
 * 
 * }
 * }
 */