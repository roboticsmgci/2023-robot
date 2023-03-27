package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.DriveConstants;;

public class ADrive extends CommandBase {
    private GenericHID m_xbox;
    private final Drivetrain m_drivetrain;
    private double y1error, x2error;

    public ADrive(GenericHID xbox, Drivetrain drivetrain) {
        m_xbox = xbox;
        m_drivetrain = drivetrain;

        setName("ADrive");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        y1error = m_xbox.getRawAxis(1);
        x2error = m_xbox.getRawAxis(4);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(getLeftSpeed(), getRightSpeed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

    /**
     * Gets the current speed of the left side of the robot.
     * @return the left speed
     */
    public double getLeftSpeed() {
        double thrust = -getAxis(1, y1error);
        if(m_xbox.getPOV()==0){
            thrust = 0.2;
        }else if(m_xbox.getPOV()==180){
            thrust = -0.2;
        }
        double twist;
        if (m_xbox.getRawButton(5)) {
            twist = 0;
        } else {
            twist = getAxis(4, x2error);
        }
        if(m_xbox.getPOV()==90){
            twist = 0.3;
        }else if(m_xbox.getPOV()==270){
            twist = -0.3;
        }
        
        double throttle = 0.9;

        if(m_xbox.getRawButton(6)){
            throttle = 0.2;
        }

        return -(-(thrust * (1 - DriveConstants.kTwistMultiplier * Math.abs(twist))
              + twist * DriveConstants.kTwistMultiplier) * throttle);

    }

    /**
     * Gets the current speed of the right side of the robot.
     * @return the right speed
     */
    public double getRightSpeed() {
        double thrust = -getAxis(1, y1error);
        if(m_xbox.getPOV()==0){
            thrust = 0.2;
        }else if(m_xbox.getPOV()==180){
            thrust = -0.2;
        }
        double twist;
        if (m_xbox.getRawButton(5)) {
            twist = 0;
        } else {
            twist = getAxis(4, x2error);
        }
        if(m_xbox.getPOV()==90){
            twist = 0.2;
        }else if(m_xbox.getPOV()==270){
            twist = -0.2;
        }
        
        double throttle = 0.9;

        if(m_xbox.getRawButton(6)){
            throttle = 0.2;
        }

        return -( -(thrust * (1 - DriveConstants.kTwistMultiplier * Math.abs(twist))
              - twist * DriveConstants.kTwistMultiplier) * throttle);

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
}
