package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn extends CommandBase {
    private final double m_degrees;
    public double m_targetAngle;
    private final Drivetrain m_drivetrain;

    private final double allowedError = 2;

    public Turn(double degrees, Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_degrees = degrees;
        
        setName("Turn");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_targetAngle = m_drivetrain.m_navX.getAngle() + m_degrees;
    }

    @Override
    public void execute() {
        double currentDegrees = m_drivetrain.m_navX.getAngle();

        if (currentDegrees < m_degrees) {
            m_drivetrain.drive(0.15, -0.15);
        } else {
            m_drivetrain.drive(-0.15, 0.15);
        }
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = m_drivetrain.m_navX.getAngle();

        return Math.abs(currentDegrees - m_targetAngle) < allowedError;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
