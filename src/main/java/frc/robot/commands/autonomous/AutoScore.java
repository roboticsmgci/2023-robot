package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmSetPID;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Turn;
import frc.robot.subsystems.*;

public class AutoScore extends SequentialCommandGroup {

    public AutoScore(int level, boolean cone, Arm arm, Intake intake) {

        addCommands(new ParallelRaceGroup(new IntakeDrive(()->true, ()->false, intake), new WaitCommand(0.3)));
        
        double armTarget = 0;
        if(level == 1){
            armTarget = 0.5;
        }else if(level == 2){
            armTarget = 0.8;
        }else if(level == 3){
            armTarget = 1.1;
        }
        addCommands(new ArmSetPID(armTarget, 3000, arm));
        addCommands(new ParallelRaceGroup(new ArmSetPID(armTarget, -2000, arm), new IntakeDrive(()-> cone, ()->!cone, intake)));
        addCommands(new ArmSetPID(0, 2000, arm));
        addCommands(new WaitCommand(1));
        setName("AutoScore");
    }
}
