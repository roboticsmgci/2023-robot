package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;

public class AutoMoveOnce extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoMoveOnce(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("AutoMoveOnce");
        addCommands(new DriveTime(2500, 0.3, drivetrain));
        
        // addCommands(new Turn(90, drivetrain));
    }
}
