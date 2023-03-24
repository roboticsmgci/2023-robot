package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

public class Drive5 extends CommandBase {
    private GenericHID m_xbox;

    private double kS=0, kV=1, kA=0;
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private double kP=0.01, kD=0;
    private PIDController m_leftPID = new PIDController(kP, 0, kD);
    private PIDController m_rightPID = new PIDController(kP, 0, kD);

    private double kPg=0.008, kDg=0.000002;
    private PIDController m_gyroPID = new PIDController(kPg, 0, kDg);

    //private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1.5);

    private double x1error, y1error, x2error, gerror;
    
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
        x1error = m_xbox.getRawAxis(0);
        y1error = m_xbox.getRawAxis(1);
        x2error = m_xbox.getRawAxis(4);
        gerror = m_drivetrain.m_navX.getAngle();
    }

    @Override
    public void execute() {
        if(m_xbox.getRawButton(5)&&m_xbox.getRawButton(6)){
            gerror = m_drivetrain.m_navX.getAngle();
        }

        double x = getAxis(0, x1error);
        double y = getAxis(1, y1error);
        double speed = (0.9*Math.max(Math.abs(x), Math.abs(y)));//Math.min(1, Math.hypot(m_xbox.getRawAxis(0), m_xbox.getRawAxis(1)));//filter.calculate(0.9*Math.min(1, m_xbox.getMagnitude()));//Math.hypot(x, y)

        // Angle between the y-axis and the direction the stick is pointed
        double angle = Math.toDegrees(Math.atan2(x, -y));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double heading = (m_drivetrain.m_navX.getAngle()-gerror)%360;
        if(heading>180){
            heading-=360;
        }

        double difference = Math.abs(angle-heading);
        if(difference>=180){
            difference = 360-difference;
        }

        if(m_xbox.getRawAxis(2)<0.5 && difference>110){
            speed*=-1;
            if(angle<0) {
                angle += 180;
            }else if(angle>0){
                angle-=180;
            }
        }
           
        double correction = MathUtil.clamp(m_gyroPID.calculate(heading, angle), -0.5, 0.5);

        double l=0, r=0;
        
        if(m_xbox.getPOV()==0){
            l+=0.15;
            r+=0.15;
        } 
        else if(m_xbox.getPOV()==180){
            l-=0.15;
            r-=0.15;
        }
        else if(m_xbox.getPOV()==90){
            l+=0.15;
            r-=0.15;
        }
        else if(m_xbox.getPOV()==270){
            l-=0.15;
            r+=0.15;
        }

        if(Math.abs(speed)>0.05){
            if(Math.abs(m_gyroPID.getPositionError())>50){
                // speed*=Math.signum(correction);
                // l+=speed;
                // r-=speed;
                l+=correction*(1+0.1*Math.abs(speed));
                r-=correction*(1+0.1*Math.abs(speed));
            }else{
                l+=speed+correction;
                r+=speed-correction;
            }
        }

        double x2 = getAxis(4, x2error);
        SmartDashboard.putNumber("x2", x2);
        if(Math.abs(x2)>0.05){
            l+=0.5*x2;
            r-=0.5*x2;
        }

        double power = Math.abs(speed)+Math.abs(correction);
        if(power>1){
            speed/=power;
            correction/=power;
        }

        m_drivetrain.drive(m_feedforward.calculate(l)+m_leftPID.calculate(m_drivetrain.m_leftLeadEncoder.getVelocity(), l), 
            m_feedforward.calculate(r)+m_rightPID.calculate(m_drivetrain.m_rightLeadEncoder.getVelocity(), r));
    }

    private double getAxis(int axis, double error){
        double value = m_xbox.getRawAxis(axis)-error;
        double cap = Math.min(1+error, 1-error);
        if(value>cap){
            value = cap;
        }else if(value<-cap){
            value = -cap;
        }

        return value;
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