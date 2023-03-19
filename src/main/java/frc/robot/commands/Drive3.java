package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

public class Drive3 extends CommandBase {
    private Joystick m_xbox;
    //private double distance;
    private double kP = 0.004, kD = 0.000002;
    
    private PDController pd = new PDController(kP, kD);

    private Drivetrain m_drivetrain;

    public Drive3(Joystick xbox, Drivetrain drivetrain) {
        m_xbox = xbox;
        m_drivetrain = drivetrain;

        setName("Drive3");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        if(m_xbox.getRawButton(5)&&m_xbox.getRawButton(6)){
            m_drivetrain.m_navX.reset();
        }

        double speed = -0.7*m_xbox.getRawAxis(1);//Math.min(1, Math.hypot(m_xbox.getRawAxis(0), m_xbox.getRawAxis(1)));//filter.calculate(0.9*Math.min(1, m_xbox.getMagnitude()));//Math.hypot(x, y)

        double angle = Math.toDegrees(Math.atan2(m_xbox.getRawAxis(2), -m_xbox.getRawAxis(3)));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double correction = 0;

        correction = pd.calculate(angle, m_drivetrain.m_navX.getAngle()%360);

        if (Math.hypot(m_xbox.getRawAxis(4), m_xbox.getRawAxis(5)) < 0.1) {
            correction = 0;
        }

        if(m_xbox.getPOV()==0){
            m_drivetrain.drive(0.3, 0.3);
        } else if(m_xbox.getPOV()==180){
            m_drivetrain.drive(-0.3, -0.3);
        }
        else if(m_xbox.getPOV()==90){
            m_drivetrain.drive(0.4, -0.4);
        }
        else if(m_xbox.getPOV()==270){
            m_drivetrain.drive(-0.4, 0.4);
        }
        // else if(m_xbox.getRawAxis(2)>0.9&&m_xbox.getRawAxis(3)>0.9){
        //     m_drivetrain.drive2(speed, correction+0.05*Math.signum(correction), true);
           
        // }
        else{
            m_drivetrain.drive2(speed, 0.4*m_xbox.getRawAxis(4), false);
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