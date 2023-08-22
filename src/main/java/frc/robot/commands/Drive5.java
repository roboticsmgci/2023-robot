package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private final double turnRange = 110;
    private boolean backward = false;

    

    private double x1error, y1error, x2error, gerror;
    private double lastEncoderValue;
    private boolean isStill;
    
    Drivetrain m_drivetrain;

    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1.5);

    public Drive5(GenericHID xbox, Drivetrain drivetrain) {
        m_xbox = xbox;
        m_drivetrain = drivetrain;

        setName("Drive5");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        x1error = m_xbox.getRawAxis(0);
        y1error = m_xbox.getRawAxis(1);
        x2error = m_xbox.getRawAxis(4);
        // SmartDashboard.putNumber("kS", kS);
        // SmartDashboard.putNumber("kV", kV);
        // SmartDashboard.putNumber("kA", kA);
        // SmartDashboard.putNumber("kP", kP);
        // SmartDashboard.putNumber("kI", kI);
        // SmartDashboard.putNumber("kD", kD);
        // SmartDashboard.putNumber("kPg", kPg);
        // SmartDashboard.putNumber("kIg", kIg);
        // SmartDashboard.putNumber("kDg", kD);
        gerror = m_drivetrain.m_navX.getAngle();
        lastEncoderValue = m_drivetrain.m_leftLeadEncoder.getPosition();
        isStill = false;
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("isStill", isStill);

        // double newKS = SmartDashboard.getNumber("kS", 0);
        // double newKV = SmartDashboard.getNumber("kV", 0);
        // double newKA = SmartDashboard.getNumber("kA", 0);

        // kP = SmartDashboard.getNumber("kP", 0);
        // kI = SmartDashboard.getNumber("kI", 0);
        // kD = SmartDashboard.getNumber("kD", 0);
        // m_leftPID.setPID(kP, kI, kD);
        // m_rightPID.setPID(kP, kI, kD);

        // kPg = SmartDashboard.getNumber("kPg", 0);
        // kIg = SmartDashboard.getNumber("kIg", 0);
        // kDg = SmartDashboard.getNumber("kDg", 0);
        // m_gyroPID.setPID(kPg, kIg, kDg);

        // if (newKS != kS || newKV != kV || newKA != kA) {
        //     kS = newKS;
        //     kV = newKV;
        //     kA = newKA;

        //     m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        // }

        if(m_xbox.getRawButton(5)&&m_xbox.getRawButton(6)){
            gerror = m_drivetrain.m_navX.getAngle();
        }

        double x = getAxis(0, x1error);
        double y = getAxis(1, y1error);
        double speed = 0.9*Math.max(Math.abs(x), Math.abs(y));

        // Angle between the y-axis and the direction the stick is pointed
        double angle = Math.toDegrees(Math.atan2(x, -y));//Math.atan(x, -y)

        m_drivetrain.angle = angle;

        double heading = 0;
        if(backward){
            heading = MathUtil.inputModulus(m_drivetrain.m_navX.getAngle()-gerror+180, -180, 180);
            speed*=-1;
        }else{
            heading = MathUtil.inputModulus(m_drivetrain.m_navX.getAngle()-gerror, -180, 180);
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
           
        double correction = MathUtil.clamp(m_drivetrain.m_gyroPID.calculate(heading, angle), -0.5, 0.5);

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
                v+=speed*Math.cos(Math.toRadians(difference));
                omega+=correction;
            }else{
                v+=speed;
                omega+=correction;
            }
        }

        double x2 = getAxis(4, x2error);
        if(Math.abs(x2)>0.05){
            omega+=0.7*x2;
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

        double l = 2.2*(v+omega);
        double r = 2.2*(v-omega);

        m_drivetrain.driveVoltage(l, r);
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