package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private double error = 0;

    private final CANSparkMax m_motor = new CANSparkMax(6, MotorType.kBrushless);

    public RelativeEncoder m_encoder = m_motor.getEncoder();

    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1);

    public double speed;    

    public Arm() {

        // Restores factory defaults, does not persist
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(true); // may need to change
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_A);

        m_encoder.setPosition(0);

        error = m_encoder.getPosition();

        // Set conversion ratios
        m_encoder.setPositionConversionFactor(0.07279);

        setName("Arm");
    }

    /**
     * Set the arm output power. Positive is out, negative is in.
     * 
     * @param speed The power (0-1)
     */
    public void setMotor(double speed) {
        m_motor.set(speed);
    }

    /**
     * Logs information to the SmartDashboard.
     */

    @Override
    public void periodic() {
        log();
    }

    public double getAngle(){
        if(error==0){
            error = m_encoder.getPosition();
        }
        return m_encoder.getPosition() - error;
    }

    public void reset(){
        error = m_encoder.getPosition();
    }

    public void log() {
        SmartDashboard.putNumber("arm", m_encoder.getPosition());
        SmartDashboard.putNumber("arm2", getAngle());
    }
}
