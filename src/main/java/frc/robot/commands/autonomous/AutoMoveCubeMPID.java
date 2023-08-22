package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.subsystems.*;

public class AutoMoveCubeMPID extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoMoveCubeMPID(Drivetrain drivetrain, Arm arm, Intake intake) {
        m_drivetrain = drivetrain;

        setName("AutoMoveCubeMPID");
        addCommands(new AutoScore(2, false, arm, intake));
        addCommands(new DriveTime(3000, -0.3, drivetrain));
        
        // addCommands(new Turn(90, drivetrain));
    }
}
