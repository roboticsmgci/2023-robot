package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;

public class SusChargeBack extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public SusChargeBack(Drivetrain drivetrain, Arm arm) {
        m_drivetrain = drivetrain;

        setName("SusChargeBack");
        addCommands(new DriveTime(2000, 0.3, drivetrain));
        addCommands(new WaitCommand(1));
        addCommands(new AutoChargeBack(drivetrain, arm));
        
        // addCommands(new Turn(90, drivetrain));
    }
}
