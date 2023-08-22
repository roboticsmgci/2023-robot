package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeDrive extends CommandBase {
    private final BooleanSupplier m_cubeIn;
    private final BooleanSupplier m_coneIn;
    private final Intake m_intake;

    int lastGamePiece;

    public IntakeDrive(BooleanSupplier cubeIn, BooleanSupplier coneIn, Intake intake) {
        m_cubeIn = cubeIn;
        m_coneIn = coneIn;
        m_intake = intake;
        
        setName("IntakeDrive");
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        lastGamePiece = GamePiece.NOTHING;
    }

    @Override
    public void execute() {
        if (m_cubeIn.getAsBoolean()) {
            // cube in or cone out
            m_intake.setMotor(IntakeConstants.INTAKE_OUTPUT_POWER, IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            lastGamePiece = GamePiece.CUBE;
        } else if (m_coneIn.getAsBoolean()) {
            // cone in or cube out
            m_intake.setMotor(-IntakeConstants.INTAKE_OUTPUT_POWER, IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            lastGamePiece = GamePiece.CONE;
        } else if (lastGamePiece == GamePiece.CUBE) {
            m_intake.setMotor(IntakeConstants.INTAKE_HOLD_POWER, IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        } else if (lastGamePiece == GamePiece.CONE) {
            m_intake.setMotor(-IntakeConstants.INTAKE_HOLD_POWER, IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        } else {
            m_intake.setMotor(0, 0);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setMotor(0, 0);
    }
}
