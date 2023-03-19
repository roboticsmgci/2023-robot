package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;

public class AutoMoveOnly extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoMoveOnly(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("AutoMoveOnly");
        addCommands(new DriveTime(2500, 0.3, drivetrain));
        addCommands(new WaitCommand(1));
        addCommands(new DriveTime(4000, -0.3, drivetrain));
        
        // addCommands(new Turn(90, drivetrain));
    }
}
