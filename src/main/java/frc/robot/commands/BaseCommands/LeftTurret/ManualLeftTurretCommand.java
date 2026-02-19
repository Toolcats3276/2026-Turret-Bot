package frc.robot.commands.BaseCommands.LeftTurret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LeftTurretSS;

public class ManualLeftTurretCommand extends Command {
    
    private LeftTurretSS s_leftTurretSS;
    private DoubleSupplier leftTurretSup;

    public ManualLeftTurretCommand(LeftTurretSS s_leftTurretSS, DoubleSupplier leftTurretSup) {
        this.s_leftTurretSS = s_leftTurretSS;
        this.leftTurretSup = leftTurretSup;
        addRequirements(s_leftTurretSS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double LeftTurretVal = MathUtil.applyDeadband(leftTurretSup.getAsDouble(), Constants.stickDeadband);
        s_leftTurretSS.Manual(LeftTurretVal);
        SmartDashboard.putNumber("LeftTurretVal", LeftTurretVal);
        SmartDashboard.putNumber("LeftTurretSup", leftTurretSup.getAsDouble());
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
