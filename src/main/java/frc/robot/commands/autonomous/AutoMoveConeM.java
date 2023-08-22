package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.subsystems.*;

public class AutoMoveConeM extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoMoveConeM(Drivetrain drivetrain, Arm arm, Intake intake) {
        m_drivetrain = drivetrain;

        setName("AutoMoveConeM");
        addCommands(new ParallelRaceGroup(new WaitCommand(1.5), new ArmDrive(() -> 0.15, arm)));
        addCommands(new ParallelRaceGroup(new WaitCommand(1), new ArmDrive(() -> 0.08, arm), new IntakeDrive(()-> true, ()->false, intake)));
        addCommands(new ParallelRaceGroup(new WaitCommand(1), new ArmDrive(() -> -0.1, arm)));
        addCommands(new DriveTime(3000, -0.3, drivetrain));
        
        // addCommands(new Turn(90, drivetrain));
    }
}
