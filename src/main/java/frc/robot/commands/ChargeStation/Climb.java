package frc.robot.commands.ChargeStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Climb extends CommandBase {
   
    private final Drivetrain m_drivetrain;

    public Climb(Drivetrain drivetrain) {
       
        m_drivetrain = drivetrain;
        
        setName("Climb");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.drive(0, 0);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(0.5, 0.5);
    }

    @Override
    public boolean isFinished() {
        return m_drivetrain.getPitch()>6;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
