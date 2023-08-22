package frc.robot.commands.ChargeStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceBack extends CommandBase {
   
    private final Drivetrain m_drivetrain;
    private double m_pitch, m_lastPitch;

    public BalanceBack(Drivetrain drivetrain) {
       
        m_drivetrain = drivetrain;
        m_pitch = -5;
        m_lastPitch = -5;
        
        setName("BalanceBack");
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
        m_pitch = m_drivetrain.getPitch();
        if(m_pitch-m_lastPitch>0.5){
            m_drivetrain.drive((0.1-m_pitch*0.01)/2, (0.1-m_pitch*0.01)/2);
        }else{
            m_drivetrain.drive(0.3, 0.3);
        }
        m_lastPitch = m_pitch;
    }

    @Override
    public boolean isFinished() {
        return m_pitch>-3;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
