package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.BaseCommands.InterpolatorShootCommand;
import frc.robot.commands.BaseCommands.SlavedTurretCommand;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.commands.BaseCommands.LeftTurret.LeftInterpolatorShoot;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretAutoAim;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretLinearActuator;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.LeftTurret.ManualLeftTurretCommand;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.ManualRightTurretCommand;
import frc.robot.commands.BaseCommands.RightTurret.RightInterpolatorShoot;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretAutoAim;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
import frc.robot.commands.ComplexCommands.ComplianceCoCommand;
import frc.robot.commands.ComplexCommands.InfeedCoCommand;
import frc.robot.commands.ComplexCommands.InfeedCoCommand2;
import frc.robot.commands.ComplexCommands.ResetTurretCoCommand;
import frc.robot.commands.ComplexCommands.ShootCoCommand;
import frc.robot.subsystems.*;



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

    private final int rightTurretSup = XboxController.Axis.kRightY.value;
    private final int leftTurretSup = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 14);
    private final JoystickButton robotCentric = new JoystickButton(driver, 0);

    private final JoystickButton Shoot = new JoystickButton(driver, 1);
    private final JoystickButton Copmliance = new JoystickButton(driver, 2);
    private final JoystickButton Infeed = new JoystickButton(driver, 3);
    private final JoystickButton Infeed2 = new JoystickButton(driver, 4);
    private final JoystickButton AutoAim = new JoystickButton(driver, 10);
    private final JoystickButton ResetTurret = new JoystickButton(driver, 13);
    private final JoystickButton InterpolatorShootTest = new JoystickButton(driver, 12);
    private final JoystickButton Test = new JoystickButton(driver, 15);

    private final JoystickButton Setpoint1 = new JoystickButton(driver, 9);
    private final JoystickButton Setpoint2 = new JoystickButton(driver, 8);
    private final JoystickButton Setpoint3 = new JoystickButton(driver, 6);
    private final JoystickButton Setpoint4 = new JoystickButton(driver, 7);


    /* Xbox Buttons */
    private final JoystickButton RightTurretManual = new JoystickButton(xboxController, 1);

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
    private final InfeedPivotSS s_InfeedPivotSS = new InfeedPivotSS();
    private final FeederSS s_Feeder = new FeederSS();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );


        AutoChooser = new SendableChooser<Command>();
        SmartDashboard.putData(AutoChooser);

        NamedCommands.registerCommand("Infeed", new InfeedCoCommand(s_Infeed, s_InfeedPivotSS));

        AutoChooser.addOption("None", new PrintCommand("No Auto??"));

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

        Shoot.whileTrue(new ShootCoCommand(s_Indexer, s_RightShooter, s_LeftShooter, s_RightTurret, s_LeftTurret, s_InfeedPivotSS, s_Infeed, s_Feeder));
        Copmliance.onTrue(new ComplianceCoCommand(s_RightShooter, s_RightTurret, s_LeftShooter, s_LeftTurret, s_Indexer, s_Infeed, s_InfeedPivotSS, s_RightShooterHood, s_LeftShooterHood, s_Feeder));
        // Infeed.onTrue(new InfeedCoCommand(s_Infeed, s_InfeedPivotSS));
        // Infeed2.onTrue(new InfeedCoCommand2(s_Infeed, s_InfeedPivotSS));
        // Infeed.onTrue(new LeftTurretPID(s_LeftTurret, -1, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED));
        Infeed.onTrue(new RightTurretAutoAim(s_RightTurret, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED_Auto));
        // Infeed2.onTrue(new LeftTurretPID(s_LeftTurret, 1, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED));
        AutoAim.onTrue(new SlavedTurretCommand(s_RightTurret, s_LeftTurret));
        ResetTurret.onTrue(new ResetTurretCoCommand(s_RightTurret, s_LeftTurret));
        // Test.onTrue(new FeederCommand(s_Feeder, RotationsPerSecond.of(1)));
        // Test.onTrue(new IndexerCommand(s_Indexer, RotationsPerSecond.of(1)));
        // Test.onTrue(new ShootLeftTurret(s_LeftShooter, 1));
        // Test.onTrue(new ShootRightTurret(s_RightShooter, 1));
        Test.onTrue(new InfeedCommand(s_Infeed, RotationsPerSecond.of(1)));

        InterpolatorShootTest.onTrue(new InterpolatorShootCommand(s_Indexer, s_Infeed, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_Feeder));

        /* Xbox Controller */
        RightTurretManual.onTrue(new ManualRightTurretCommand(s_RightTurret, () -> xboxController.getRawAxis(rightTurretSup)));
        RightTurretManual.onTrue(new ManualLeftTurretCommand(s_LeftTurret, () -> xboxController.getRawAxis(leftTurretSup)));


        Setpoint4.onTrue(new RightTurretLinearActuator(s_RightShooterHood, Millimeter.of(.5)));
        // TurretReset.onTrue(new RightTurretPID(s_RightShooterHood, 0, 1));
        // TurretReset.onTrue(new LeftTurretPID(s_LeftShooterHood, .1, 1));
        // TurretReset.onTrue(new InstantCommand(() -> s_RightTurret.LinearActuator(1)));
        // TurretReset.onTrue(new InstantCommand(() -> s_LeftTurret.LinearActuator(1)));
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
