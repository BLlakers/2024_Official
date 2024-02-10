package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Hanger extends SubsystemBase{
    public CANSparkMax hangerLeftMtr = new CANSparkMax(Constants.hangerLeftMtrC,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  public CANSparkMax hangerRightMtr = new CANSparkMax(Constants.hangerRightMtrC,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    public RelativeEncoder hangerLeftMtrEnc = hangerLeftMtr.getEncoder();
    public RelativeEncoder hangerRightMtrEnc = hangerRightMtr.getEncoder(); 

    public Supplier<Rotation3d> m_robotOrientationSupplier;
    public PIDController m_hangingController = new PIDController(1, 0, 0);

    private static final double GOAL_HEIGHT = 0.5; // meters
    private static final double GOAL_ORIENTATION = 0.0; // radians


    public Hanger (Supplier<Rotation3d> robotOrientationSupplier){
        m_robotOrientationSupplier = robotOrientationSupplier;
    }

    @Override

    public void periodic() {
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Hanger/Left Hang Pos", hangerLeftMtrEnc.getPosition());
        SmartDashboard.putNumber("Hanger/Right Hang Pos", hangerRightMtrEnc.getPosition());

        SmartDashboard.putNumber("Hanger/Left Current",hangerLeftMtr.getOutputCurrent());
        SmartDashboard.putNumber("Hanger/Right Current",hangerRightMtr.getOutputCurrent());

        Rotation3d robotOrientation = m_robotOrientationSupplier.get();

        double roll, pitch, yaw;
        roll  = robotOrientation.getX(); // radians
        pitch = robotOrientation.getY(); // radians 
        yaw   = robotOrientation.getZ(); // radians

        SmartDashboard.putNumber("Robot/Navx/Orientation/Roll",  Units.radiansToDegrees(roll));
        SmartDashboard.putNumber("Robot/Navx/Orientation/Pitch", Units.radiansToDegrees(pitch));
        SmartDashboard.putNumber("Robot/Navx/Orientation/Yaw",   Units.radiansToDegrees(yaw));
    }

    public Command LeftHangUp() {
        return run(
            () -> {
            hangerLeftMtr.set(1);
            });
      }

    public Command LeftHangDown() {
        return run(
            () -> {
            hangerLeftMtr.set(-1);
            });
      }

    public Command LeftHangStop() {
        return run(
            () -> {
            hangerLeftMtr.set(0);
            });
      }
    public Command RightHangUp() {
        return run(
            () -> {
            hangerRightMtr.set(1);
            });
      }

    public Command RightHangDown() {
        return run(
            () -> {
            hangerRightMtr.set(-1);
            });
      }

    public Command RightHangStop() {
        return run(
            () -> {
            hangerRightMtr.set(0);
            });
      }

}
