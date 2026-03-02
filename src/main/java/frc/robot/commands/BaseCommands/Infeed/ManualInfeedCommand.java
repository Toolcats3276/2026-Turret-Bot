package frc.robot.commands.BaseCommands.Infeed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.InfeedPivotSS;

public class ManualInfeedCommand extends Command {
    
    private InfeedPivotSS s_InfeedPivot;
    private DoubleSupplier infeedSup;

    public ManualInfeedCommand(InfeedPivotSS s_InfeedPivot, DoubleSupplier infeedSup) {
        this.s_InfeedPivot = s_InfeedPivot;
        this.infeedSup = infeedSup;
        addRequirements(s_InfeedPivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double InfeedVal = MathUtil.applyDeadband(infeedSup.getAsDouble(), Constants.stickDeadband);
        s_InfeedPivot.Manual(InfeedVal);
        SmartDashboard.putNumber("InfeedVal", InfeedVal);
        SmartDashboard.putNumber("InfeedSup", infeedSup.getAsDouble());
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
