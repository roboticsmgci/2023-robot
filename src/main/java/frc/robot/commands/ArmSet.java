package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmSet extends CommandBase {
    private final double m_target;
    private final Arm m_arm;

    public ArmSet(double target, Arm arm) {
        m_target = target;
        m_arm = arm;

        setName("ArmSet");
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        double error = m_target-m_arm.m_encoder.getPosition();
        //if(error)
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setMotor(0);
    }
}
