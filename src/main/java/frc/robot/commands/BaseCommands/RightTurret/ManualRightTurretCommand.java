package frc.robot.commands.BaseCommands.RightTurret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RightTurretSS;

public class ManualRightTurretCommand extends Command {
    
    private RightTurretSS s_rightTurretSS;
    private DoubleSupplier rightTurretSup;

    public ManualRightTurretCommand(RightTurretSS s_rightTurretSS, DoubleSupplier rightTurretSup) {
        this.s_rightTurretSS = s_rightTurretSS;
        this.rightTurretSup = rightTurretSup;
        addRequirements(s_rightTurretSS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double RightTurretVal = MathUtil.applyDeadband(rightTurretSup.getAsDouble(), Constants.stickDeadband);
        s_rightTurretSS.Manual(RightTurretVal);
        SmartDashboard.putNumber("RightTurretVal", RightTurretVal);
        SmartDashboard.putNumber("RightTurretSup", rightTurretSup.getAsDouble());
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
