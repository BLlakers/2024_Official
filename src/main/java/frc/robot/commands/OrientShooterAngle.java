package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Shooter;

public class OrientShooterAngle extends ProfiledPIDCommand {
    private Shooter m_Shooter;

    private static double s_kP = 0.5;
    private static double s_kI = 0.0;
    private static double s_kD = 0.0;
    
    private static final double s_maxShooterAnglingVelocity     = Shooter.s_angleMotorSpeedPercentage; // motor percentage power
    private static final double s_maxShooterAnglingAcceleration = s_maxShooterAnglingVelocity / 2;     // motor percentage power

    public OrientShooterAngle(Shooter shooter, Rotation2d goal) {
        super(
                new ProfiledPIDController(s_kP, s_kI, s_kD, new TrapezoidProfile.Constraints(s_maxShooterAnglingVelocity, s_maxShooterAnglingAcceleration)),
                () -> {
                    return shooter.GetShooterAngle().getRadians();
                },
                goal.getRadians(),
                (output, setpoint) -> shooter.SetShooterAngleSpeedPercentage(output),
                shooter);
                
        getController().enableContinuousInput(-Math.PI, Math.PI);
        m_Shooter = shooter;
    }

    @Override
    public void initialize() {
        super.initialize();
        getController().setTolerance(Units.degreesToRadians(0.5)); // set 0.5 degree tolerance for error
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.AngleStop();
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

}
