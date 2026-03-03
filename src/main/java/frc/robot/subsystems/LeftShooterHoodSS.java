package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.vision.LimelightAssistant;


public class LeftShooterHoodSS extends SubsystemBase{

    private Servo s_LinearActuator;

    private Distance shootangle;
    
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
              s_LinearActuator.set(.075);
              break;
            }

        }

        SmartDashboard.putNumber("Left Linear Acutator Angle", s_LinearActuator.get());
        // SmartDashboard.putNumber("Left Linear Actuator Setpoint", shootangle.in(Millimeter));

    }

    public void Stop(){
        HoodMode = Mode.stop;
    }

    public void LinearActuator(Distance shootangle){
        this.shootangle = shootangle;
        HoodMode = Mode.LinearActuator;
    }

    public Distance LinearActuatorSetPoint(){
        return shootangle;
    }

}


