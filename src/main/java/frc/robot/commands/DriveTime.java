package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTime extends CommandBase {
    private double m_time;
    private final double m_ms;
    private final double m_speed;
    private final Drivetrain m_drivetrain;

    private double m_distanceCounter;

    public DriveTime(double ms, double speed, Drivetrain drivetrain) {
        m_ms = ms;
        m_time = 0;
        m_speed = speed;
        m_drivetrain = drivetrain;
        
        setName("DriveTime");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.drive(0, 0);
        // m_drivetrain.m_leftLeadEncoder.setPosition(0);
        // m_drivetrain.m_rightLeadEncoder.setPosition(0);
        //m_drivetrain.m_Encoder.setPosition(0);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(m_speed, m_speed);
        m_time+=20;
    }

    @Override
    public boolean isFinished() {
        // m_distanceCounter = (- m_drivetrain.m_leftLeadEncoder.getPosition() 
        //                      + m_drivetrain.m_rightLeadEncoder.getPosition())
        //                     / 2;
        return m_time>=m_ms;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
