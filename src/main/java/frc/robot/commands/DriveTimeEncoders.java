package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTimeEncoders extends CommandBase {
    private double m_time;
    private final double m_ms;
    private final double m_speed;
    private final double m_turnSpeed;
    private final Drivetrain m_drivetrain;

    private final double kp = 0.008;
    private final double ki = 0;
    private final double kd = 0.000002;

    PIDController pid = new PIDController(kp, ki, kd);

    /**
     * 
     * @param ms milliseconds duration to drive for
     * @param speed speed between 0 and 1 - turnSpeed
     * @param turnSpeed maximum speed at which to turn in response to pid
     * @param drivetrain drivetrain
     */
    public DriveTimeEncoders(double ms, double speed, double turnSpeed, Drivetrain drivetrain) {
        m_ms = ms;
        m_time = 0;
        m_speed = speed;
        m_turnSpeed = turnSpeed;
        m_drivetrain = drivetrain;
        
        setName("DriveTimeEncoders");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.drive(0, 0);
        double initialDifference = m_drivetrain.m_leftLeadEncoder.getPosition() - m_drivetrain.m_rightLeadEncoder.getPosition();
        pid.setSetpoint(initialDifference);
    }

    @Override
    public void execute() {
        double difference = m_drivetrain.m_leftLeadEncoder.getPosition() - m_drivetrain.m_rightLeadEncoder.getPosition();
        double correction = MathUtil.clamp(pid.calculate(difference), -m_turnSpeed, m_turnSpeed);
        m_drivetrain.drive(m_speed - correction, m_speed + correction);
        m_time+=20;
    }

    @Override
    public boolean isFinished() {
        return m_time>=m_ms;
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
