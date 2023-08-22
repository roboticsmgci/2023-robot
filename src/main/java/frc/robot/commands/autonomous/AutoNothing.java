package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;

public class AutoNothing extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoNothing(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("AutoNothing");
    }
}
