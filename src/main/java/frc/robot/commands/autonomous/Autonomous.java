package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public Autonomous(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("Autonomous");
        addCommands(new DriveDistance(1, 0.4, drivetrain));
        addCommands(new DriveDistance(-0.01, 0.2, drivetrain));
        // addCommands(new Turn(90, drivetrain));
    }
}
