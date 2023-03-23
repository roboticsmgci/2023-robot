package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

public class Drive5 extends CommandBase {
    private GenericHID m_xbox;

    private double kS=0, kV=0, kA=0;
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private double kP=0.01, kD=0;
    private PIDController m_leftPID = new PIDController(kP, 0, kD);
    private PIDController m_rightPID = new PIDController(kP, 0, kD);

    private double kPg=0.008, kDg=0.000002;
    private PIDController m_gyroPID = new PIDController(kPg, 0, kDg);

    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1.5);
    
Drivetrain m_drivetrain;

    public Drive5(GenericHID xbox, Drivetrain drivetrain) {
        m_xbox = xbox;
        m_drivetrain = drivetrain;

        m_gyroPID.enableContinuousInput(-180, 180);

        setName("Drive5");
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

        double speed = m_speedLimiter.calculate(0.9*Math.max(Math.abs(m_xbox.getRawAxis(0)), Math.abs(m_xbox.getRawAxis(1))));//Math.min(1, Math.hypot(m_xbox.getRawAxis(0), m_xbox.getRawAxis(1)));//filter.calculate(0.9*Math.min(1, m_xbox.getMagnitude()));//Math.hypot(x, y)

        // Angle between the y-axis and the direction the stick is pointed
        double angle = Math.toDegrees(Math.atan2(m_xbox.getRawAxis(0), -m_xbox.getRawAxis(1)));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double heading = m_drivetrain.m_navX.getAngle();

        if(!m_xbox.getRawButton(1) && Math.abs((heading-angle)%360)>90){
            speed*=-1;
            heading+=180;
        }
           
        double correction = m_gyroPID.calculate(heading, angle);

        double l=0, r=0;
        
        if(m_xbox.getPOV()==0){
            m_drivetrain.drive(0.15, 0.15);
        } else if(m_xbox.getPOV()==180){
            m_drivetrain.drive(-0.15, -0.15);
        }
        else if(m_xbox.getPOV()==90){
            m_drivetrain.drive(0.15, -0.15);
        }
        else if(m_xbox.getPOV()==270){
            m_drivetrain.drive(-0.15, 0.15);
        }
        if(Math.abs((heading-angle)%360)>50){
            speed*=Math.signum(correction);
            m_drivetrain.drive2(0, speed);
        }else{
            double power = speed*(1+correction);
            if(power>1){
                speed/=power;
                correction/=power;
            }
            l+=speed+correction;
            r+=speed-correction;
        }
        if(Math.abs(m_xbox.getRawAxis(4))>0.1){
            l+=0.5*m_xbox.getRawAxis(4);
            r-=0.5*m_xbox.getRawAxis(4);
        }
        m_drivetrain.drive(m_feedforward.calculate(l)+m_leftPID.calculate(m_drivetrain.m_leftLeadEncoder.getVelocity(), l), 
            m_feedforward.calculate(l)+m_rightPID.calculate(m_drivetrain.m_rightLeadEncoder.getVelocity(), r));
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