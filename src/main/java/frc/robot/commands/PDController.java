package frc.robot.commands;

public class PDController {
    private double kP, kD, error, lastError;

    public PDController(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
        error = 0;
        lastError = 0;
    }
    
    public double calculate(double setpoint, double measurement){
        
        this.lastError = this.error;
        error = setpoint - measurement;
        if(error>180){
            error-=360;
        }else if(error<-180){
            error+=360;
        }

        return kP*error+kD*(error-lastError);
    }
}
