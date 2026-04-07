package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.commands.BaseCommands.AutoShootAtTargetCommand;
import frc.robot.commands.BaseCommands.LookAtTargetCommand;
import frc.robot.commands.BaseCommands.ShootAtTargetCommand;
import frc.robot.commands.BaseCommands.SmartCompCommand;
import frc.robot.commands.BaseCommands.SmartInfeedCommand;
import frc.robot.commands.ComplexCommands.CancelCoCommand;
import frc.robot.commands.ComplexCommands.ComplianceCoCommand;
import frc.robot.commands.ComplexCommands.InfeedCoCommand;
import frc.robot.commands.ComplexCommands.InfeedCoCommand2;
import frc.robot.commands.ComplexCommands.InterpolatorShootCoCommand;
import frc.robot.commands.ComplexCommands.OutFeed;
import frc.robot.commands.ComplexCommands.ShuttleCoCommand;
import frc.robot.subsystems.*;

import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Hub_SetPoints_Left;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Hub_SetPoints_Right;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Shuttle_SetPoints_Left;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Shuttle_SetPoints_Right;
import static frc.robot.Constants.RobotConstants.VisionConstants.TARGET_BLUE;
import static frc.robot.Constants.RobotConstants.VisionConstants.TARGET_RED;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.RobotConstants.VisionConstants.AUTO_SHOOTER_BARRIER_BLUE;
import static frc.robot.Constants.RobotConstants.VisionConstants.AUTO_SHOOTER_BARRIER_RED;

import static frc.robot.Constants.RobotConstants.VisionConstants.SHUTTLE_BLUE_LEFT;
import static frc.robot.Constants.RobotConstants.VisionConstants.SHUTTLE_BLUE_RIGHT;
import static frc.robot.Constants.RobotConstants.VisionConstants.SHUTTLE_RED_RIGHT;
import static frc.robot.Constants.RobotConstants.VisionConstants.SHUTTLE_RED_LEFT;

import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> AutoChooser;
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController xboxController = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
    private final JoystickButton robotCentric = new JoystickButton(driver, 0);

    private final JoystickButton Shoot = new JoystickButton(driver, 1);
    private final JoystickButton Copmliance = new JoystickButton(driver, 2);
    private final JoystickButton Infeed = new JoystickButton(driver, 3);
    private final JoystickButton Infeed2 = new JoystickButton(driver, 4);
    private final JoystickButton Cancel = new JoystickButton(driver, 10);
    private final JoystickButton Shuttle = new JoystickButton(driver, 9);
    private final JoystickButton xDrive = new JoystickButton(driver, 8);

    private final JoystickButton LookatTarget = new JoystickButton(driver, 6);
    private final JoystickButton Outfeed = new JoystickButton(driver, 7);
    private final JoystickButton SmartComp = new JoystickButton(driver, 0);

    /* Subsystems */
    private final SwerveSS s_Swerve = new SwerveSS();
    private final LeftTurretSS s_LeftTurret = new LeftTurretSS();
    private final LeftShooterSS s_LeftShooter = new LeftShooterSS();
    private final LeftShooterHoodSS s_LeftShooterHood = new LeftShooterHoodSS();
    private final RightTurretSS s_RightTurret = new RightTurretSS();
    private final RightShooterSS s_RightShooter = new RightShooterSS();
    private final RightShooterHoodSS s_RightShooterHood = new RightShooterHoodSS();
    private final IndexerSS s_Indexer = new IndexerSS();
    private final InfeedSS s_Infeed = new InfeedSS();
    private final InfeedPivotSS s_InfeedPivot = new InfeedPivotSS();
    private final FeederSS s_Feeder = new FeederSS();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> LookatTarget.getAsBoolean()
            )
        );


        AutoChooser = new SendableChooser<Command>();
        SmartDashboard.putData(AutoChooser);

        NamedCommands.registerCommand("Infeed", new SmartInfeedCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, s_InfeedPivot, 
            targetSelector -> {
              Translation2d target;
              if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters) && targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)) {
                if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
                } else {
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
                }
              } else {
                if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    target = TARGET_BLUE;
                }
                else {
                    target = TARGET_RED;
                }
              }
            return target;
            }));
        NamedCommands.registerCommand("Shoot", new AutoShootAtTargetCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, s_InfeedPivot, 
            targetSelector -> {
              Translation2d target;
            //   if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters) && targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)) {
            //     if (DriverStation.getAlliance().orElse(Blue) == Blue) {
            //         target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
            //     } else {
            //         target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
            //     }
            //   } else {
                if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    target = TARGET_BLUE;
                }
                else {
                    target = TARGET_RED;
                }
            //   }
            return target;
            }));
        NamedCommands.registerCommand("Comp", new SmartCompCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, 
            targetSelector -> {
              Translation2d target;
              if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters) && targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)) {
                if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
                } else {
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
                }
              } else {
                if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    target = TARGET_BLUE;
                }
                else {
                    target = TARGET_RED;
                }
              }
            return target;
            }));
        NamedCommands.registerCommand("ZeroGyro", new InstantCommand(() -> s_Swerve.zeroHeading()));
        NamedCommands.registerCommand("Infeed2", new InfeedCoCommand2(s_Infeed, s_InfeedPivot));

        AutoChooser.addOption("None", new PrintCommand("No Auto??"));
        // AutoChooser.addOption("Depot", new PathPlannerAuto("Depot"));
        AutoChooser.addOption("PID", new PathPlannerAuto("PID"));
        AutoChooser.addOption("Left Mid To Depot", new PathPlannerAuto("Left Mid To Depot"));
        // AutoChooser.addOption("Left To Mid", new PathPlannerAuto("Left Mid To Depot"));
        // AutoChooser.addOption("Right Mid to Depot", new PathPlannerAuto("Right Mid to Depot"));
        AutoChooser.addOption("Shuttle", new PathPlannerAuto("Shuttle"));
        // AutoChooser.addOption("Test", new PathPlannerAuto("Test"));

        // Configure the button bindings

        
        configureButtonBindings();

    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        Shoot.onTrue(new ShootAtTargetCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, s_InfeedPivot, 
            targetSelector -> {
              Translation2d target;
              if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                  if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters)) {
                      target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
                  }
                  else{
                      target = TARGET_BLUE;
                  }
              }
              else{
                  if(targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)){
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
                  }
                  else{
                    target = TARGET_RED;
                  }
                }
            return target;
            }));

        // Copmliance.onTrue(new ComplianceCoCommand(s_RightShooter, s_RightTurret, s_LeftShooter, s_LeftTurret, s_Indexer, s_Infeed, s_InfeedPivot, s_RightShooterHood, s_LeftShooterHood, s_Feeder));

        // Infeed.onTrue(new InfeedCoCommand(s_Infeed, s_InfeedPivot));
        Infeed.onTrue(new SmartInfeedCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, s_InfeedPivot, 
            targetSelector -> {
              Translation2d target;
              if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                  if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters)) {
                      target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
                  }
                  else{
                      target = TARGET_BLUE;
                  }
              }
              else{
                  if(targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)){
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
                  }
                  else{
                    target = TARGET_RED;
                  }
                }
            return target;
            }));
        Infeed2.onTrue(new InfeedCoCommand2(s_Infeed, s_InfeedPivot));

        Outfeed.onTrue(new OutFeed(s_Infeed, s_Feeder, s_Indexer));
        Cancel.onTrue(new CancelCoCommand(s_Indexer, s_InfeedPivot, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_LeftTurret, s_RightTurret, s_Infeed, s_Feeder));

        xDrive.whileTrue(Commands.run(() -> s_Swerve.Xdrive(true), s_Swerve));
        
        // LookatTarget.onTrue(new LookAtTargetCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, t -> DriverStation.getAlliance().orElse(Blue) == Blue ? TARGET_BLUE : TARGET_RED, Shuttle_SetPoints_By_Limelight_Degrees_Right, Shuttle_SetPoints_By_Limelight_Degrees_Left));
        // Shuttle.onTrue(new ShootAtTargetCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, s_InfeedPivot,         
        //     targetSelector -> {
        //       Translation2d target;
        //       if (DriverStation.getAlliance().orElse(Blue) == Blue) {
        //         target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
        //       } else {
        //         target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
        //       }
        //     //   // Adjust the target to be "offset distance" short of target along the vector between the robot and the target
        //     //   Translation2d vectorToTarget = target.minus(shooterTranslation);
        //     //   double distanceToTarget = vectorToTarget.getNorm();
        //     //   Translation2d adjustedTarget = shooterTranslation
        //     //       .plus(vectorToTarget.times((distanceToTarget - SHUTTLE_OFFSET_DISTANCE.in(Meters)) / distanceToTarget));
        //     //   return adjustedTarget;
        //     return target;
        //     }, Shuttle_SetPoints_Right, Shuttle_SetPoints_Left));

        Copmliance.onTrue(new SmartCompCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, 
            targetSelector -> {
              Translation2d target;
              if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                  if (targetSelector.getX() > AUTO_SHOOTER_BARRIER_BLUE.in(Meters)) {
                      target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_LEFT : SHUTTLE_BLUE_RIGHT;
                  }
                  else{
                      target = TARGET_BLUE;
                  }
              }
              else{
                  if(targetSelector.getX() < AUTO_SHOOTER_BARRIER_RED.in(Meters)){
                    target = targetSelector.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_RIGHT : SHUTTLE_RED_LEFT;
                  }
                  else{
                    target = TARGET_RED;
                  }
                }
            return target;
            }));

        // LookatTarget.onTrue(new LookAtTargetCommand(s_RightShooter, s_RightTurret, s_RightShooterHood, s_LeftShooter, s_LeftTurret, s_LeftShooterHood, s_Feeder, s_Indexer, s_Infeed, s_Swerve, t -> DriverStation.getAlliance().orElse(Blue) == Blue ? TARGET_BLUE : TARGET_RED, Hub_SetPoints_Right, Hub_SetPoints_Left));
        // Shuttle.onTrue(new ShuttleCoCommand(s_Indexer, s_RightTurret, s_LeftTurret, s_Infeed, s_InfeedPivot, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_Feeder));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        return AutoChooser.getSelected();
    }
}
