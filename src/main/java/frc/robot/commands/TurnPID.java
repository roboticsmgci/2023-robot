package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnPID extends CommandBase {
    private final double m_degrees;
    private final Drivetrain m_drivetrain;
    private final PIDController pid;

    private double initialAngle;
    private double targetAngle;

    private final double kp = 0.008;
    private final double ki = 0;
    private final double kd = 0.000002;
    private final double error = 2;
    private final double errorD = 0.2;

    public TurnPID(double degrees, Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_degrees = degrees;

        pid = new PIDController(kp, ki, kd);
        pid.setTolerance(error, errorD);
        
        setName("Turn PID");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        initialAngle = m_drivetrain.m_navX.getAngle();
        targetAngle = initialAngle + m_degrees;

        pid.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("isFinished", 0);
        double speed = MathUtil.clamp(pid.calculate(m_drivetrain.m_navX.getAngle()), -0.25, 0.25);
        SmartDashboard.putNumber("gyro distance", pid.getPositionError());
        m_drivetrain.drive(speed, -speed);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("isFinished", 1);
        return pid.atSetpoint();
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}
