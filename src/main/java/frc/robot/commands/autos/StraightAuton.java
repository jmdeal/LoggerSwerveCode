package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class StraightAuton extends SequentialCommandGroup {
    private SwerveDrivetrain m_Drivetrain;
    PathPlannerTrajectory straightBackTraj;

    public StraightAuton(SwerveDrivetrain dt) {
        m_Drivetrain = dt;

        straightBackTraj = PathPlanner.loadPath("straight path", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);
        
        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(
            AutonConstants.kPThetaController, 0, 0, AutonConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand straightBackCommand = new SwerveControllerCommand(
            straightBackTraj, 
            m_Drivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            translationController, 
            strafeController, 
            thetaController, 
            m_Drivetrain::setModuleStates, 
            m_Drivetrain);

        addCommands(new InstantCommand(
                () -> m_Drivetrain.resetOdometry(straightBackTraj.getInitialHolonomicPose())),
                        straightBackCommand);
    }

}
