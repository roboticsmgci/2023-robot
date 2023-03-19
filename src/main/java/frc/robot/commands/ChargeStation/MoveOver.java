package frc.robot.commands.ChargeStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveOver extends CommandBase {
   
    private final Drivetrain m_drivetrain;
    private double m_pitch, m_lastPitch;

    public MoveOver(Drivetrain drivetrain) {
       
        m_drivetrain = drivetrain;
        m_pitch = 0;
        m_lastPitch = 0;
        
        setName("MoveOver");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_pitch = m_drivetrain.getPitch();
        m_lastPitch = m_drivetrain.getPitch();
        m_drivetrain.drive(0, 0);
    }

    @Override
    public void execute() {
        m_lastPitch = m_pitch;
        m_pitch = m_drivetrain.getPitch();
        m_drivetrain.drive(-0.3, -0.3);
    }

    @Override
    public boolean isFinished() {
        return m_pitch > 0 && m_pitch<5 && m_lastPitch-m_pitch>0.4;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
