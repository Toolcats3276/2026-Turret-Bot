package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RightShooterHoodSS extends SubsystemBase{

    private Servo s_LinearActuator;

    private double shootangle;
    
    public RightShooterHoodSS() {
        
        s_LinearActuator = new Servo(8);
        s_LinearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

     public enum Mode{
        LinearActuator,
        stop
    }

    Mode HoodMode = Mode.stop;
    
    @Override

    public void periodic() {

        switch(HoodMode) {

            case LinearActuator:{
                s_LinearActuator.set(shootangle);
                break;
            }

            case stop:{
              s_LinearActuator.set(.075);
              break;
            }

        }

        SmartDashboard.putNumber("Right Linear Acutator Angle", s_LinearActuator.get());
        SmartDashboard.putNumber("Right Linear Actuator Setpoint", shootangle);

    }

    public void LinearActuator(double shootangle){
        this.shootangle = shootangle;
        HoodMode = Mode.LinearActuator;
    }

    public double LinearActuatorSetPoint(){
        return shootangle;
    }

}


