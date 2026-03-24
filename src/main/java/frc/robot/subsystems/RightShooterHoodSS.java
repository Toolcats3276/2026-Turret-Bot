package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;

import static frc.robot.Constants.RobotConstants.FrontRightTurret.RightShooterHoodConversion;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.FUEL_EXIT_ANGLE_OFFSET;



public class RightShooterHoodSS extends SubsystemBase{

    private Servo s_LinearActuator;

    private Distance shootangle;
    
    public RightShooterHoodSS() {
        
        s_LinearActuator = new Servo(6);
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

        SmartDashboard.putNumber("Right Linear Acutator Angle", s_LinearActuator.get());
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

    public static record RightShooterConversion(
        Distance shotAngle,
        Angle outputAngle) {
        
        public RightShooterConversion interpolate(RightShooterConversion endValue, double t) {
          RightShooterConversion result = new RightShooterConversion(
            Millimeters.of(MathUtil.interpolate(shotAngle.in(Millimeters), endValue.shotAngle.in(Millimeters), t)),
            Degrees.of(
                  MathUtil.interpolate(
                      outputAngle.in(Degrees),
                        endValue.outputAngle.in(Degrees),
                        t))
        
            );
          return result;
        }
    }

    public Angle getFuelPitch(Angle shooterPitch) {
        // The hood moving up (positive) lowers the exit angle, so the angle has to be subtracted.
        // Turret 0 shoots fuel at FUEL_EXIT_ANGLE_OFFSET
        return FUEL_EXIT_ANGLE_OFFSET.minus(shooterPitch);
    }  
}




