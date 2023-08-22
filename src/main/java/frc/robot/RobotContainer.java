// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.ArmDrive1;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Drive5;
import frc.robot.commands.Drive5Sim;
import frc.robot.commands.ADrive;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");

    public final Drivetrain m_drivetrain = new Drivetrain();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();

    Joystick m_stick1 = new Joystick(0);
    XboxController m_xbox = new XboxController(0);
    XboxController m_xbox2 = new XboxController(1);

    Compressor pcmCompressor = new Compressor(9, PneumaticsModuleType.CTREPCM);

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(){
        // pcmCompressor.enableDigital();        

        inst.startServer();

        // Configure the trigger bindings
        configureBindings();

        CameraServer.startAutomaticCapture();

        m_chooser.addOption("Auto nothing", new AutoNothing(m_drivetrain));
        m_chooser.addOption("Auto only move forward", new AutoMoveOnce(m_drivetrain));
        m_chooser.setDefaultOption("Auto move both ways", new AutoMoveOnly(m_drivetrain));
        m_chooser.addOption("Auto sus charge", new SusChargeBack(m_drivetrain, m_arm));
        m_chooser.addOption("Auto cube mid back", new AutoMoveCubeM(m_drivetrain, m_arm, m_intake));
        m_chooser.addOption("Auto cube mid back better", new AutoMoveCubeMPID(m_drivetrain, m_arm, m_intake));
        m_chooser.addOption("Auto cube mid only", new AutoScore(2, false, m_arm, m_intake));
        m_chooser.addOption("Auto cone mid back", new AutoMoveConeM(m_drivetrain, m_arm, m_intake));
        m_chooser.addOption("Auto charge forwards", new AutoChargeOnly(m_drivetrain));
        m_chooser.addOption("Auto charge backwards", new AutoChargeBack(m_drivetrain, m_arm));
        m_chooser.addOption("Auto charge move", new AutoChargeMove(m_drivetrain));
        
        SmartDashboard.putData(m_chooser);

        m_drivetrain.setDefaultCommand(
            //new ADrive(m_stick1, m_drivetrain)
            new Drive5(m_stick1, m_drivetrain)
        );

        m_arm.setDefaultCommand(
            new ArmDrive1(m_xbox2, m_arm)
        );
        // m_arm.setDefaultCommand(new ArmDrive(() -> 0.05, m_arm));

        m_intake.setDefaultCommand(new IntakeDrive(
            () -> {
                return m_xbox2.getRawButton(6);
                // return true;
                // return m_xbox2.getRawButton(4);
            },
            () -> {
                return m_xbox2.getRawAxis(3) > 0.9;
                // return m_xbox2.getRawAxis(2) > 0.9;
            },
            m_intake
        ));

        // try{
        //     Socket socket = new Socket("wpilibpi.local", 5801);
        //     InputStream input = socket.getInputStream();
        //     int character;
        //     StringBuilder data = new StringBuilder();
        //     InputStreamReader reader = new InputStreamReader(input);
        //      while((character = reader.read()) != -1){
        //         data.append((char)character);
        //      }

        //      SmartDashboard.putString("data: ", ""+data);
        // }catch (IOException e){
        //     SmartDashboard.putString("Exception: ", ""+e.toString());
        // }
    }

    /**
     * Gets the current speed of the left side of the robot.
     * @return the left speed
     */
    public double getLeftSpeed() {
        double thrust = -m_xbox.getRawAxis(1);
        if(m_xbox.getPOV()==0){
            thrust = 0.2;
        }else if(m_xbox.getPOV()==180){
            thrust = -0.2;
        }
        double twist;
        if (m_xbox.getLeftBumper()) {
            twist = 0;
        } else {
            twist = m_xbox.getRawAxis(4);
        }
        if(m_xbox.getPOV()==90){
            twist = 0.3;
        }else if(m_xbox.getPOV()==270){
            twist = -0.3;
        }
        
        double throttle = 0.9;

        if(m_xbox.getRawButton(6)){
            throttle = 0.2;
        }

        return -(-(thrust * (1 - DriveConstants.kTwistMultiplier * Math.abs(twist))
              + twist * DriveConstants.kTwistMultiplier) * throttle);

    }

    /**
     * Gets the current speed of the right side of the robot.
     * @return the right speed
     */
    public double getRightSpeed() {
        double thrust = -m_xbox.getRawAxis(1);
        if(m_xbox.getPOV()==0){
            thrust = 0.2;
        }else if(m_xbox.getPOV()==180){
            thrust = -0.2;
        }
        double twist;
        if (m_xbox.getLeftBumper()) {
            twist = 0;
        } else {
            twist = m_xbox.getRawAxis(4);
        }
        if(m_xbox.getPOV()==90){
            twist = 0.2;
        }else if(m_xbox.getPOV()==270){
            twist = -0.2;
        }
        
        double throttle = 0.9;

        if(m_xbox.getRawButton(6)){
            throttle = 0.2;
        }

        return -( -(thrust * (1 - DriveConstants.kTwistMultiplier * Math.abs(twist))
              - twist * DriveConstants.kTwistMultiplier) * throttle);

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // frc2::JoystickButton(&m_stick2,2).WhenHeld(
        //     SpinPropeller(m_propeller)
        // );
        //new JoystickButton(m_stick1, DriveConstants.kTurnButton).onTrue(new Turn(90, m_drivetrain));
        // new JoystickButton(m_xbox, ArmConstants.EXTEND_BUTTON)
        //     .whileTrue(new ArmDrive(() -> ArmConstants.OUTPUT_POWER, m_arm));
        // new JoystickButton(m_xbox, ArmConstants.RETRACT_BUTTON)
        //     .whileTrue(new ArmDrive(() -> -ArmConstants.OUTPUT_POWER, m_arm));
        // new JoystickButton(m_xbox, 4)
        // .onTrue(new TurnPID(180, m_drivetrain));
        //     new JoystickButton(m_xbox, 3)
        //     .onTrue(new TurnPID(135, m_drivetrain));
        // new JoystickButton(m_xbox, 2)
        //     .onTrue(new TurnPID(90, m_drivetrain));
        // new JoystickButton(m_xbox, 1)
        //     .onTrue(new TurnPID(45, m_drivetrain));
        // new JoystickButton(m_xbox, 2)
        //     .onTrue(new DriveDistance(1, 0.2, m_drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // var autoVoltageConstraint =
        // new DifferentialDriveVoltageConstraint(
        //     m_drivetrain.m_feedforward,
        //     m_drivetrain.kDriveKinematics,
        //     10);

        // // Create config for trajectory
        // TrajectoryConfig config =
        //     new TrajectoryConfig(
        //             3,
        //             3)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(m_drivetrain.kDriveKinematics)
        //         // Apply the voltage constraint
        //         .addConstraint(autoVoltageConstraint);

        // // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         // Pass config
        //         config);

        // RamseteCommand ramseteCommand =
        //     new RamseteCommand(
        //         exampleTrajectory,
        //         m_robotDrive::getPose,
        //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        //         new SimpleMotorFeedforward(
        //             DriveConstants.ksVolts,
        //             DriveConstants.kvVoltSecondsPerMeter,
        //             DriveConstants.kaVoltSecondsSquaredPerMeter),
        //         DriveConstants.kDriveKinematics,
        //         m_robotDrive::getWheelSpeeds,
        //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //         // RamseteCommand passes volts to the callback
        //         m_robotDrive::tankDriveVolts,
        //         m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
        // // An example command will be run in autonomous
        // //return m_chooser.getSelected();
        return null;
    }
}
