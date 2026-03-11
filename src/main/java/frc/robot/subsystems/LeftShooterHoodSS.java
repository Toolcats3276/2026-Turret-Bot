package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LeftShooterHoodSS extends SubsystemBase{

    private Servo s_LinearActuator;

    private Distance shootangle;
    private Distance Tolerance = Millimeter.of(.001);
    
    public LeftShooterHoodSS() {
        
        s_LinearActuator = new Servo(9);
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
                s_LinearActuator.set(shootangle.in(Millimeter));
                break;
            }

            case stop:{
              s_LinearActuator.set(0);
              break;
            }

        }


        SmartDashboard.putNumber("Left Linear Acutator Angle", s_LinearActuator.getPosition());
        SmartDashboard.putBoolean("Left LinearActuator in range", LinearActuatorInRange());
        SmartDashboard.putNumber("Lower Limit", LinearActuatorLowerSetPoint());
        SmartDashboard.putNumber("Higher Limit", LinearActuatorHigherSetPoint());
        SmartDashboard.putNumber("Limit", LinearActuatorSetPoint());

        LinearActuatorInRange();
    }

    public void Stop(){
        HoodMode = Mode.stop;
    }

    public void LinearActuator(Distance shootangle){
        this.shootangle = shootangle;
        HoodMode = Mode.LinearActuator;
    }

    public double LinearActuatorSetPoint(){
        return s_LinearActuator.getPosition();
    }

    public double LinearActuatorLowerSetPoint(){
        return LinearActuatorSetPoint() - 0.0015;
    }
    
    public double LinearActuatorHigherSetPoint(){
        return LinearActuatorSetPoint() + 0.0015;
    }

    public boolean LinearActuatorInRange(){
        boolean Inrange;
        if (LinearActuatorSetPoint()> LinearActuatorLowerSetPoint()) {
            Inrange = true;
        }

        else if (LinearActuatorSetPoint() < LinearActuatorHigherSetPoint()) {
            Inrange = true;
        }
        else{
            Inrange = false;
        }
        return Inrange;
    }
}


