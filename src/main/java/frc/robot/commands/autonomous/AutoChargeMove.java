package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.commands.ChargeStation.*;
import frc.robot.subsystems.Drivetrain;

public class AutoChargeMove extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoChargeMove(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("Auto Charge Move");
        addCommands(new MoveOver(drivetrain));
        addCommands(new AutoChargeOnly(drivetrain));
    }
}
