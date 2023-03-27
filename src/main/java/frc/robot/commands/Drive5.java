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

/*
 * Controls
 * Left joystick: field oriented drive
 * Right joystick x axis: turn (clockwise or counter clockwise)
 * Left bumper: slow (25% speed)
 * Left trigger: lock direction (always go forward)
 * Right trigger: go straight
 * Left bumper + right bumper: reset gyro
 * Dpad: slow arcade drive (4 directions)
 */

public class Drive5 extends CommandBase {
    private GenericHID m_xbox;

    private final double followRange = 60;
    private final double turnRange = 100;

    private double kS=0.22, kV=2.91, kA=0;
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private double kP=0.0001, kD=0;
    private PIDController m_leftPID = new PIDController(kP, 0, kD);
    private PIDController m_rightPID = new PIDController(kP, 0, kD);

    private double kPg=0.022, kDg=0.000002;
    private PIDController m_gyroPID = new PIDController(kPg, 0, kDg);

    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1.5);

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
        double speed = 0.9*Math.max(Math.abs(x), Math.abs(y));

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

        if(m_xbox.getRawAxis(3)<0.5 && difference>turnRange){
            speed*=-1;
            if(angle<0) {
                angle += 180;
            }else if(angle>0){
                angle-=180;
            }

            difference = Math.abs(angle-heading);
            if(difference>=180){
                difference = 360-difference;
            }
        }
           
        double correction = MathUtil.clamp(m_gyroPID.calculate(heading, angle), -0.5, 0.5);

        if(m_xbox.getRawAxis(2)>0.5 || Math.abs(speed) < 0.05){
            correction = 0;
        }

        double v=0, omega=0;
        
        if(m_xbox.getPOV()==0){
            v+=0.15;
        } 
        else if(m_xbox.getPOV()==180){
            v-=0.15;
        }
        else if(m_xbox.getPOV()==90){
            omega+=0.15;
        }
        else if(m_xbox.getPOV()==270){
            omega-=0.15;
        }

        if(Math.abs(speed)>0.05){
            if(difference>followRange){
                v+=speed*Math.cos(difference);
                omega+=correction;
            }else{
                v+=speed;
                omega+=correction;
            }
        }

        double x2 = getAxis(4, x2error);
        if(Math.abs(x2)>0.05){
            omega+=0.5*x2;
        }

        if(m_xbox.getRawButton(5)){
            v*=0.25;
            omega*=0.5;
        }

        v = m_speedLimiter.calculate(v);

        double power = Math.abs(v)+Math.abs(omega);
        if(power>1){
            v/=power;
            omega/=power;
        }

        double l = 1.5*(v+omega);
        double r = 1.5*(v-omega);

        m_drivetrain.driveVoltage(m_feedforward.calculate(l)+m_leftPID.calculate(m_drivetrain.m_leftLeadEncoder.getVelocity(), l), 
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