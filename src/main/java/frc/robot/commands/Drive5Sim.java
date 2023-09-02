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

public class Drive5Sim extends CommandBase {
    private GenericHID m_xbox;

    private final double followRange = 60;
    private final double turnRange = 130;
    private boolean backward = false;

    private final double kS=0.02, kV=1, kA=0;
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private final double kP=0.02, kD=0;
    private final PIDController m_leftPID = new PIDController(kP, 0, kD);
    private final PIDController m_rightPID = new PIDController(kP, 0, kD);

    private final double kPg=0.008, kDg=0.000002;
    private final PIDController m_gyroPID = new PIDController(kPg, 0, kDg);

    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1.5);

    private double x1error, y1error, x2error, gerror;
    
Drivetrain m_drivetrain;

    public Drive5Sim(GenericHID xbox, Drivetrain drivetrain) {
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
        //gerror = m_drivetrain.m_navX.getAngle();
        m_drivetrain.resetGyro();
    }

    @Override
    public void execute() {
        if(m_xbox.getRawButton(5)&&m_xbox.getRawButton(6)){
            m_drivetrain.resetGyro();
            // gerror = m_drivetrain.m_navX.getAngle()%360;
        }

        double x = getAxis(0, x1error);
        double y = -getAxis(1, y1error);
        double speed = Math.max(Math.abs(x), Math.abs(y));

        // Angle between the y-axis and the direction the stick is pointed
        double angle = Math.toDegrees(Math.atan2(x, y));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double heading = 0;
        if(backward){
            heading = MathUtil.inputModulus(m_drivetrain.getYaw()+180, -180, 180);
            speed*=-1;
        }else{
            heading = MathUtil.inputModulus(m_drivetrain.getYaw(), -180, 180);
        }

        double difference = Math.abs(angle-heading);
        if(difference>=180){
            difference = 360-difference;
        }

        if(difference>turnRange){
            backward = !backward;
        }
        if(m_xbox.getRawAxis(3)>0.5){
            backward = false;
        }

        double correction;

        correction = MathUtil.clamp(m_drivetrain.m_gyroPID.calculate(heading, angle), -0.5, 0.5);

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

        if(m_xbox.getRawAxis(2)>0.8){
            correction = 0;
        }

        if(Math.abs(speed)>0.05){
            v+=speed*Math.cos(Math.toRadians(difference));
                omega+=correction;
            // if(difference>followRange){
            //     v+=speed*Math.cos(Math.toRadians(difference));
            //     omega+=correction;
            // }else{
            //     v+=speed;
            //     omega+=correction;
            // }
        }

        double x2 = getAxis(4, x2error);
        if(Math.abs(x2)>0.05){
            omega+=0.7*x2;
        }

        if(m_xbox.getRawAxis(2)>0.8){
            v=Math.signum(y);
        }

        if(m_xbox.getRawButton(5)){
            v*=0.20;
            omega*=0.5;
        }

        v = m_speedLimiter.calculate(v);

        double power = Math.abs(v)+Math.abs(omega);
        if(power>1){
            v/=power;
            omega/=power;
        } 

        double l = 2.4*(v+omega);
        double r = 2.4*(v-omega);

        // m_drivetrain.drive(m_feedforward.calculate(l)+m_leftPID.calculate(m_drivetrain.m_leftLeadEncoder.getVelocity(), l), 
        //     m_feedforward.calculate(r)+m_rightPID.calculate(m_drivetrain.m_rightLeadEncoder.getVelocity(), r));
        m_drivetrain.drive(m_feedforward.calculate(l), 
            m_feedforward.calculate(r));
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