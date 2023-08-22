package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

public class Drive4 extends CommandBase {
    private Joystick m_xbox;
    private double m_initialAngle;

    //private double distance;
    private final double kp = 0.008;
    private final double ki = 0;
    private final double kd = 0.000002;
    
    private PIDController pid = new PIDController(kp, ki, kd);

    private Drivetrain m_drivetrain;

    public Drive4(Joystick xbox, Drivetrain drivetrain) {
        m_xbox = xbox;
        m_drivetrain = drivetrain;

        setName("Drive4");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        m_initialAngle = m_drivetrain.m_navX.getAngle();
    }

    @Override
    public void execute() {
        if(m_xbox.getRawButton(5)&&m_xbox.getRawButton(6)){
            m_initialAngle = m_drivetrain.m_navX.getAngle();
        }

        double speed = -0.7*m_xbox.getRawAxis(1);//Math.min(1, Math.hypot(m_xbox.getRawAxis(0), m_xbox.getRawAxis(1)));//filter.calculate(0.9*Math.min(1, m_xbox.getMagnitude()));//Math.hypot(x, y)

        double angle = Math.toDegrees(Math.atan2(m_xbox.getRawAxis(4), -m_xbox.getRawAxis(5)));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double currentGyro = m_drivetrain.m_navX.getAngle();

        double correction = MathUtil.clamp(pid.calculate(currentGyro, angle), -0.3, 0.3);

        if (Math.hypot(m_xbox.getRawAxis(4), m_xbox.getRawAxis(5)) < 0.1) {
            correction = 0;
        }

        if(m_xbox.getPOV()==0){
            m_drivetrain.drive(0.12, 0.12);
        } else if(m_xbox.getPOV()==180){
            m_drivetrain.drive(-0.12, -0.12);
        }
        else if(m_xbox.getPOV()==90){
            m_drivetrain.drive(0.12, -0.12);
        }
        else if(m_xbox.getPOV()==270){
            m_drivetrain.drive(-0.12, 0.12);
        }else if(m_xbox.getRawAxis(2)>0.5&&m_xbox.getRawAxis(3)>0.5){
            //m_drivetrain.drive2(speed, correction, true);
           
        }
        else{
            //m_drivetrain.drive2(speed, 0.25*m_xbox.getRawAxis(4), true);
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