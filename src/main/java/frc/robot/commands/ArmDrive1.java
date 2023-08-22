package frc.robot.commands;

import java.lang.annotation.Target;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;

public class ArmDrive1 extends CommandBase {
    private GenericHID m_xbox;
    private double axisError;
    private final Arm m_arm;
    private final PIDController m_posPID = new PIDController(0.4, 0, 0);
    private final PIDController m_vPID = new PIDController(0.005, 0, 0);

    public ArmDrive1(GenericHID xbox, Arm arm) {
        m_xbox = xbox;
        m_arm = arm;

        setName("ArmDrive1");
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
        axisError = m_xbox.getRawAxis(1);
    }

    @Override
    public void execute() {
        double correction = 0;
        double target = 0;
        double angle = m_arm.getAngle();

        if(m_xbox.getRawButton(5) && m_xbox.getRawAxis(2)>0.9){
            m_arm.reset();
        }

        if(m_xbox.getRawButton(4)){
            target = 1.3;
            correction = MathUtil.clamp(m_posPID.calculate(angle, target), -0.17, 0.17)+0.05;
        }else if(m_xbox.getRawButton(3)){
            target = 1.0;
            correction = MathUtil.clamp(m_posPID.calculate(angle, target), -0.17, 0.17)+0.05;
        }
        else if(m_xbox.getRawButton(1)){
            target = 0;
            correction = MathUtil.clamp(m_posPID.calculate(angle, target), -0.17, 0.17)+0.05;
        }
        
        if(Math.abs(getAxis(1, axisError)) > 0.1){
            correction = -1*getAxis(1, axisError);
        }
        
        m_arm.setMotor(correction);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setMotor(0);
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
