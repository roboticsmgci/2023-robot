package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.commands.ChargeStation.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;

public class AutoChargeBack extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;
    private Arm m_arm;

    public AutoChargeBack(Drivetrain drivetrain, Arm arm) {
        m_drivetrain = drivetrain;
        m_arm = arm;

        setName("Auto Charge Only");
        addCommands(new ParallelRaceGroup(
            new SequentialCommandGroup(new ClimbBack(m_drivetrain),
            new BalanceBack(m_drivetrain))), 
            new ArmDrive(() -> -0.05, m_arm));
        
        // addCommands(new DriveDistance(-0.05, 0.08, drivetrain));
        addCommands(new WaitCommand(0.5));
        addCommands(new Turn(90, m_drivetrain));
    }
}
