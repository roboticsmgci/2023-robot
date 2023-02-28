package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroDrive extends CommandBase {
    private double m_x, m_y;
    private Drivetrain m_drivetrain;

    // Initialises the command with its name and requirements
    public GyroDrive(double x, double y, Drivetrain drivetrain) {

        m_x = x;
        m_y = y;
        m_drivetrain = drivetrain;

        setName("GyroDrive");
        addRequirements(m_drivetrain);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        //double radius = Math.sqrt(m_x*m_x+m_y*m_y);
        double angle = 0;
        if(m_x>0 && m_y>0){
            angle = Math.atan(m_y/m_x);
        }else if(m_x<0 && m_y>0){
            angle = 3.14159265359-Math.atan(-m_y/m_x);
        }else if(m_x<0 && m_y<0){
            angle = 3.14159265359+Math.atan(m_y/m_x);
        }else if(m_x>0 && m_y<0){
            angle = 2*3.14159265359-Math.atan(m_y/m_x);
        }
        angle*=180/3.14159265359;
        if(Math.abs(m_drivetrain.m_navX.getAngle()%360-angle) > 10){
            m_drivetrain.drive(0.2, -0.2);
        }else{
            m_drivetrain.drive(0.2, 0.2);
        }
        m_drivetrain.log();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }
}


