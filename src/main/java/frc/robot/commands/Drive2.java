package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive2 extends CommandBase {
    private Joystick m_joystick;
    //private double distance;
    private double kP = 0.01;
    private Drivetrain m_drivetrain;

    public Drive2(Joystick joystick, Drivetrain drivetrain) {
        m_joystick = joystick;
        m_drivetrain = drivetrain;

        setName("Drive2");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        m_drivetrain.m_navX.reset();
    }

    @Override
    public void execute() {
        double x = m_joystick.getX();
        double y = m_joystick.getY();

        double angle = Math.toDegrees(Math.atan(y/x));
        if(x<0){
            angle += 180;
        }
        angle-=90;
        if(angle>180){
            angle-=360;
        }
        angle*=-1;
        
        double heading = m_drivetrain.m_navX.getAngle();
        if(heading>180){
            heading-=360;
        }

        double error = kP*(heading-angle);
        double speed = 0.5*Math.sqrt(x*x+y*y);
        if(Math.abs(error)>0.5){
            m_drivetrain.drive(-speed, speed);
        }else{
            m_drivetrain.drive(speed*(1-error), speed*(1+error));
        }
        
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
