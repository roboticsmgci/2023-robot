package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public Autonomous(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("Autonomous");
        addCommands(new DriveBack(drivetrain));
    }
}
