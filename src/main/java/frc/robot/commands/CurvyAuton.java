package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class CurvyAuton extends SequentialCommandGroup {
    SwerveDrivetrain mDrivetrain;
    PathPlannerTrajectory curveTraj;
    public CurvyAuton(SwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;

        curveTraj = PathPlanner.loadPath("Curvy Auton", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(
            AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController, AutonConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand mDriveCommand = 
        new SwerveControllerCommand(
            curveTraj, 
            mDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            translationController, 
            strafeController, 
            thetaController, 
            mDrivetrain::setModuleStates, 
            mDrivetrain);

        addCommands(new InstantCommand(
            () -> mDrivetrain.resetOdometry(curveTraj.getInitialHolonomicPose())),
                    mDriveCommand)  ;
    }
}
