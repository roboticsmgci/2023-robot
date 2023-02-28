package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;
    private final Drivetrain m_drivetrain;

    public TankDrive(DoubleSupplier left, DoubleSupplier right, Drivetrain drivetrain) {
        m_left = left;
        m_right = right;
        m_drivetrain = drivetrain;

        setName("TankDrive");
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(m_left.getAsDouble(), m_right.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }
}
