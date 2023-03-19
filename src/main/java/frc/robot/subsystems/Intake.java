package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;

public class Intake extends SubsystemBase {
    CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushed); // port may not be correct

    int m_lastGamePiece;

    public Intake() {
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(false); // may need to change
        m_motor.setIdleMode(IdleMode.kBrake);

        m_lastGamePiece = GamePiece.NOTHING;

        setName("Intake");
    }

    /**
     * Set the arm output power.
     * 
     * @param percent desired speed
     * @param amps current limit
     */
    public void setMotor(double percent, int amps) {
        m_motor.set(percent);
        System.out.println(percent);
        // SmartDashboard.putNumber("intake power (%)", percent);
        // SmartDashboard.putNumber("intake motor current (amps)", m_motor.getOutputCurrent());
        // SmartDashboard.putNumber("intake motor temperature (C)", m_motor.getMotorTemperature());
    }

    @Override
    public void periodic() {
        
    }

}
