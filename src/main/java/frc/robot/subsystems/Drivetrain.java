package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    public double angle = 0;

    private final CANSparkMax m_leftLeadMotor = new CANSparkMax(DriveConstants.kLeftLeadDeviceID,
                                                                MotorType.kBrushless);
    private final CANSparkMax m_rightLeadMotor = new CANSparkMax(DriveConstants.kRightLeadDeviceID,
                                                                 MotorType.kBrushless);
    private final CANSparkMax m_leftFollowMotor = new CANSparkMax(DriveConstants.kLeftFollowDeviceID,
                                                                  MotorType.kBrushless);
    private final CANSparkMax m_rightFollowMotor = new CANSparkMax(DriveConstants.kRightFollowDeviceID,
                                                                   MotorType.kBrushless);


    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

    private double m_pitchError;
    // Gyro
    public AHRS m_navX = new AHRS(SerialPort.Port.kUSB1);

    // Encoders
    public RelativeEncoder m_leftLeadEncoder = m_leftLeadMotor.getEncoder();
    public RelativeEncoder m_rightLeadEncoder = m_rightLeadMotor.getEncoder();

    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1);

    private SlewRateLimiter m_brakeLimiter = new SlewRateLimiter(1.8);
    private SlewRateLimiter m_turnLimiter = new SlewRateLimiter(1.8);

    public Drivetrain() {

        // Restores factory defaults, does not persist
        m_leftLeadMotor.restoreFactoryDefaults();
        m_rightLeadMotor.restoreFactoryDefaults();
        m_leftFollowMotor.restoreFactoryDefaults();
        m_rightFollowMotor.restoreFactoryDefaults();

        // Inverts one side of the drivetrain
        m_rightLeadMotor.setInverted(true);
        // m_leftFollowMotor.setInverted(true);

        // Configures the motors to follow each other
        m_leftFollowMotor.follow(m_leftLeadMotor);
        m_rightFollowMotor.follow(m_rightLeadMotor);

        m_robotDrive.setDeadband(0.06);

        // Set conversion ratios
        // m_leftLeadEncoder.setPositionConversionFactor(0.0443);
        // m_rightLeadEncoder.setPositionConversionFactor(0.0443);
        m_leftLeadEncoder.setPositionConversionFactor(-0.058726117);
        m_rightLeadEncoder.setPositionConversionFactor(-0.058726117);

        m_pitchError = 0;

        setName("Drivetrain");
    }

    /**
     * Drives the robot with the given speed for each side.
     * 
     * @param left The speed of the left side.
     * @param right The speed of the right speed.
     */
    public void drive(double left, double right) {
        m_robotDrive.tankDrive(left, right, false);
    }

    public void drive(double left, double right, boolean limit) {
        drive2((left+right)/2, (left-right)/2, limit);
    }

    public void drive2(double speed, double turn, boolean limit) {
        // if(limit){
        //     speed = m_speedLimiter.calculate(speed);
        // }else{
        //     speed = m_brakeLimiter.calculate(speed);
        // }
        // turn = m_turnLimiter.calculate(turn);
        drive(speed+turn, speed - turn);
    }

    /**
     * Logs information to the SmartDashboard.
     */
    public void log() {
        SmartDashboard.putNumber("Gyro", m_navX.getYaw());
        SmartDashboard.putNumber("Angle", m_navX.getAngle());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("r", m_navX.getPitch());
        SmartDashboard.putNumber("target", angle);
        SmartDashboard.putNumber("l1", m_leftLeadEncoder.getPosition()); // encoders
        SmartDashboard.putNumber("l2", m_rightLeadEncoder.getPosition());
    }

    @Override
    public void periodic() {
        log();
    }

    public double getPitch(){
        if(m_pitchError==0){
            m_pitchError = m_navX.getPitch();
        }
        return -(m_navX.getPitch()-m_pitchError);
    }

    public void setBrakes(IdleMode idleMode) {
        m_leftLeadMotor.setIdleMode(idleMode);
        m_rightLeadMotor.setIdleMode(idleMode);
        m_leftFollowMotor.setIdleMode(idleMode);
        m_rightFollowMotor.setIdleMode(idleMode);
    }
}
