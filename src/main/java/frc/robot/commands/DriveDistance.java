package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    private final double m_distance;
    private final double m_speed;
    private final Drivetrain m_drivetrain;

    private double m_distanceCounter;

    public DriveDistance(double distance, double speed, Drivetrain drivetrain) {
        m_distance = distance;
        m_speed = speed;
        m_drivetrain = drivetrain;
        
        setName("DriveDistance");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.drive(0, 0);
        m_drivetrain.m_leftLeadEncoder.setPosition(0);
        m_drivetrain.m_rightLeadEncoder.setPosition(0);
        m_distanceCounter = 0;
    }

    @Override
    public void execute() {
        m_distanceCounter = (- m_drivetrain.m_leftLeadEncoder.getPosition() 
                             + m_drivetrain.m_rightLeadEncoder.getPosition())
                            / 2;

        if (m_distance >= 0 && m_distanceCounter < m_distance){
            m_drivetrain.drive(m_speed, m_speed);
        } else if (m_distance < 0 && m_distanceCounter > m_distance){
            m_drivetrain.drive(-m_speed, -m_speed);
        }
    }

    @Override
    public boolean isFinished() {
        m_distanceCounter = (- m_drivetrain.m_leftLeadEncoder.getPosition() 
                             + m_drivetrain.m_rightLeadEncoder.getPosition())
                            / 2;

        if (m_distance >= 0) {
            return m_distanceCounter >= m_distance;
        } else {
            return m_distanceCounter <= m_distance;
        }
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
