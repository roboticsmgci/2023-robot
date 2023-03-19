package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class ArmSetPID extends CommandBase {
    private final double m_degrees;
    private final Arm m_arm;
    private final PIDController pid;

    private double initialAngle;
    private double targetAngle;

    private final double kp = 0.1;
    private final double ki = 0.02;
    private final double kd = 0.2;
    private final double error = 2;
    private final double errorD = 5;

    public ArmSetPID(double degrees, Arm arm) {
        m_arm = arm;
        m_degrees = degrees;

        pid = new PIDController(kp, ki, kd);
        pid.setTolerance(error, errorD);
        
        setName("Arm Set PID");
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        initialAngle = m_arm.m_encoder.getPosition();
        targetAngle = initialAngle + m_degrees;

        pid.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        double speed = pid.calculate(m_arm.m_encoder.getPosition());
        m_arm.setMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_arm.setMotor(0);
    }

}
