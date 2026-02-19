package frc.robot.commands.BaseCommands.Infeed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.InfeedSS;

public class ManualInfeedCommand extends Command {
    
    private InfeedSS s_Infeed;
    private DoubleSupplier infeedSup;

    public ManualInfeedCommand(InfeedSS s_Infeed, DoubleSupplier infeedSup) {
        this.s_Infeed = s_Infeed;
        this.infeedSup = infeedSup;
        addRequirements(s_Infeed);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double InfeedVal = MathUtil.applyDeadband(infeedSup.getAsDouble(), Constants.stickDeadband);
        s_Infeed.Manual(InfeedVal);
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
