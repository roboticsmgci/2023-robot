package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    // SIMULATION
    DifferentialDrivetrainSim m_driveSim;

    int m_dev;
    SimDouble m_simAngle;

    Encoder m_leftSimEncoder;
    Encoder m_rightSimEncoder;
    EncoderSim m_leftEncoderSim;
    EncoderSim m_rightEncoderSim;

    Field2d m_field = new Field2d();
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        m_navX.getRotation2d(),
        m_leftLeadEncoder.getPosition(), m_rightLeadEncoder.getPosition()
    );

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
        // SmartDashboard.putNumber("target", angle);
        SmartDashboard.putNumber("l1", m_leftLeadEncoder.getPosition()); // encoders
        SmartDashboard.putNumber("l2", m_rightLeadEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // System.out.println(m_leftLeadEncoder.getPosition());
        m_odometry.update(m_navX.getRotation2d(),
                    m_leftLeadEncoder.getPosition(),
                    m_rightLeadEncoder.getPosition());
        m_field.setRobotPose(m_odometry.getPoseMeters());
        // System.out.print(m_odometry.getPoseMeters().getX());
        // System.out.print(", ");
        // System.out.println(m_odometry.getPoseMeters().getY());
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

    public void simInit() {
        m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            7.29,
            7.5,
            60.0,
            Units.inchesToMeters(3),
            0.7112,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );

        m_dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        m_simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_dev, "Yaw"));
        m_simAngle.set(0);
        
        SmartDashboard.putData("Field", m_field);

        // REVPhysicsSim.getInstance().addSparkMax(m_leftLeadMotor, DCMotor.getNEO(1));
        // REVPhysicsSim.getInstance().addSparkMax(m_leftFollowMotor, DCMotor.getNEO(1));
        // REVPhysicsSim.getInstance().addSparkMax(m_rightLeadMotor, DCMotor.getNEO(1));
        // REVPhysicsSim.getInstance().addSparkMax(m_rightFollowMotor, DCMotor.getNEO(1));
        m_leftSimEncoder = new Encoder(0, 1);
        m_rightSimEncoder = new Encoder(2, 3);
        m_leftSimEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(3) / 4096);
        m_rightSimEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(3) / 4096);
        m_leftEncoderSim = new EncoderSim(m_leftSimEncoder);
        m_rightEncoderSim = new EncoderSim(m_rightSimEncoder);
    }

    public void simUpdate() {
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        // System.out.print(m_leftLeadMotor.get() * RobotController.getInputVoltage());
        // System.out.print(", ");
        // System.out.println(m_rightLeadMotor.get() * RobotController.getInputVoltage());
        m_driveSim.setInputs(m_leftLeadMotor.get() * RobotController.getInputVoltage(),
                       m_rightLeadMotor.get() * RobotController.getInputVoltage());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_driveSim.update(0.02);

        // Update all of our sensors.
        // m_leftLeadEncoder.setPosition(m_leftLeadEncoder.getPosition() + 0.1 * m_leftLeadMotor.get());
        // m_rightLeadEncoder.setPosition(m_rightLeadEncoder.getPosition() + 0.1 * m_rightLeadMotor.get());
        // REVPhysicsSim.getInstance().run();
        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        m_leftLeadEncoder.setPosition(m_driveSim.getLeftPositionMeters());
        m_rightLeadEncoder.setPosition(m_driveSim.getRightPositionMeters());
       
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        gyroSimAngle.set(-m_driveSim.getHeading().getDegrees());
    }
}
