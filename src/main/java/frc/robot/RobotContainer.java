// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ChaseTag;
import frc.robot.commands.CurvyAuton;
import frc.robot.commands.HardCurveAuton;
import frc.robot.commands.StraightAuton;
import frc.robot.commands.StraightBackAuton;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModuleIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private final XboxController m_driverController = new XboxController(JoystickConstants.DRIVER_PORT_ID);

  PhotonCamera camera = new PhotonCamera("gloworm");
  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_zeroMods = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_trackAprilTag = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
 
  
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain(
    new GyroIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO(){}, new SwerveModuleIO(){});

  private final ChaseTag mChaseTag = new ChaseTag(camera, m_swerveDrivetrain);

  private final CurvyAuton mCurvyAuton = new CurvyAuton(m_swerveDrivetrain);
  private final StraightBackAuton mStraightBackAuton = new StraightBackAuton(m_swerveDrivetrain);
  private final StraightAuton mStraightAuton = new StraightAuton(m_swerveDrivetrain);
  private final HardCurveAuton mHardCurveAuton = new HardCurveAuton(m_swerveDrivetrain);
  private static LoggedDashboardChooser<Command> mAutonChooser = new LoggedDashboardChooser<Command>("Auto Chooser");;
  
  public RobotContainer() {
    // Configure the button bindings
    boolean fieldRelative = true;
    boolean openLoop = false;
    m_swerveDrivetrain.setDefaultCommand(new TeleopDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));
    
    mAutonChooser.addDefaultOption("Curvy Default Path", mCurvyAuton);
    mAutonChooser.addOption("Straight Back Auton", mStraightBackAuton);
    // LoggedDashboard.putData("Auton Mode Chooser", mAutonChooser);
    configureButtonBindings();

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_zeroGyro.onTrue(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));
    m_trackAprilTag.whileTrue(mChaseTag);
    //m_zeroMods.whenPressed(new InstantCommand(() -> m_swerveDrivetrain.zeroModules()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return mCurvyAuton;
  }
}
