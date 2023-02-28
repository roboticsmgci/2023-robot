package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private double m_speed;
    //private double distance;
    private double kP = 0.01;
    private double heading;
    private Drivetrain m_drivetrain;

    public Drive(double speed, Drivetrain drivetrain) {
        m_speed = speed;
        m_drivetrain = drivetrain;

        setName("Drive");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        heading = m_drivetrain.m_navX.getAngle() % 360;
    }

    @Override
    public void execute() {
        double error = kP*((m_drivetrain.m_navX.getAngle() % 360)-heading);
        m_drivetrain.drive(m_speed*(1-error), m_speed*(1+error));
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
