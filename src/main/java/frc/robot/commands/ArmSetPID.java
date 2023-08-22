package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;

public class ArmSetPID extends CommandBase {
    private final Arm m_arm;
    private final PIDController pid;

    private double targetAngle;
    private double m_ms;
    private double m_time;
    private boolean stop=true;

    private final double kp = 0.65;
    private final double ki = 0.0;
    private final double kd = 0.0;
    private final double error = 0.05;
    private final double errorD = 5;

    public ArmSetPID(double target, int ms, Arm arm) {
        m_arm = arm;
        targetAngle = target;
        m_ms = ms;
        m_time = 0;

        if(m_ms<0){
            m_ms*=-1;
            stop=false;
        }

        pid = new PIDController(kp, ki, kd);
        pid.setTolerance(error, errorD);
        
        setName("Arm Set PID");
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        double speed = MathUtil.clamp(pid.calculate(m_arm.m_encoder.getPosition()), -0.17, 0.17)+0.05;
        m_arm.setMotor(speed);
        m_time+=20;
    }

    @Override
    public boolean isFinished() {
        return (pid.atSetpoint()&&stop) || m_time>=m_ms;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_arm.setMotor(0);
    }

}
