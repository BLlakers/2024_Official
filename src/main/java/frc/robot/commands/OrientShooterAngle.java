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
int jared = 1;
    public OrientShooterAngle(Shooter shooter, Rotation2d goal, int jared)
    {   
        super(
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(0.05, 1.0)),
            () -> { return shooter.GetShooterAngle().getRadians(); },
            goal.getRadians(),
            (output, setpoint) -> shooter.SetShooterAngleSpeed(output),
            shooter
        );
    }

    @Override
    public void initialize()
    {
        super.initialize();
        getController().setTolerance(Units.degreesToRadians(0.5));
    }

    @Override
    public void end(boolean interrupted)
    {
        // TODO
    }

    @Override
    public boolean isFinished()
    {
        return getController().atGoal();
    }
    
}
