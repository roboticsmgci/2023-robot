package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax m_leftLeadMotor = new CANSparkMax(DriveConstants.kLeftLeadDeviceID,
                                                                MotorType.kBrushed);
    private final CANSparkMax m_rightLeadMotor = new CANSparkMax(DriveConstants.kRightLeadDeviceID,
                                                                 MotorType.kBrushed);
    private final CANSparkMax m_leftFollowMotor = new CANSparkMax(DriveConstants.kLeftFollowDeviceID,
                                                                  MotorType.kBrushed);
    private final CANSparkMax m_rightFollowMotor = new CANSparkMax(DriveConstants.kRightFollowDeviceID,
                                                                   MotorType.kBrushed);

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

    // Gyro
    public AHRS m_navX = new AHRS(SPI.Port.kMXP);

    public Drivetrain() {

        // Restores factory defaults, does not persist
        m_leftLeadMotor.restoreFactoryDefaults();
        m_rightLeadMotor.restoreFactoryDefaults();
        m_leftFollowMotor.restoreFactoryDefaults();
        m_rightFollowMotor.restoreFactoryDefaults();

        // Inverts one side of the drivetrain
        m_leftLeadMotor.setInverted(true);
        // m_leftFollowMotor.setInverted(true);

        // Configures the motors to follow each other
        m_leftFollowMotor.follow(m_leftLeadMotor);
        m_rightFollowMotor.follow(m_rightLeadMotor);

        setName("Drivetrain");

    }

    /**
     * Drives the robot with the given speed for each side.
     * 
     * @param left The speed of the left side.
     * @param right The speed of the right speed.
     */
    public void drive(double left, double right) {
        m_robotDrive.tankDrive(left, right);
    }

    /**
     * Logs information to the SmartDashboard.
     */
    public void log() {
        SmartDashboard.putNumber("Gyro", m_navX.getYaw());
    }

    @Override
    public void periodic() {
        log();
    }

}
