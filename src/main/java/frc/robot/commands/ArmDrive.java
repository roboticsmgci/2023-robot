package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmDrive extends CommandBase {
    private final DoubleSupplier m_speed;
    private final Arm m_arm;

    public ArmDrive(DoubleSupplier speed, Arm arm) {
        m_speed = speed;
        m_arm = arm;

        setName("ArmDrive");
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.setMotor(m_speed.getAsDouble());
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
