package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Turn;
import frc.robot.commands.ChargeStation.*;
import frc.robot.subsystems.Drivetrain;

public class AutoChargeOnly extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;

    public AutoChargeOnly(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setName("Auto Charge Only");
        addCommands(new Climb(drivetrain));
        //addCommands(new DriveDistance(0.5, 0.4, m_drivetrain));
        addCommands(new Balance(drivetrain));
        // addCommands(new DriveDistance(-0.05, 0.08, drivetrain));
        addCommands(new DriveTime(500, 0, drivetrain));
        addCommands(new Turn(90, m_drivetrain));
    }
}
