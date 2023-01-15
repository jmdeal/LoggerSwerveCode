package frc.robot.commands.autos;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class JPathAuton extends SequentialCommandGroup {
    SwerveDrivetrain mDrivetrain;
    PathPlannerTrajectory jTraj;
    public JPathAuton(SwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;

        jTraj = PathPlanner.loadPath("j path", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(
            AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController, AutonConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand mDriveCommand = 
        new SwerveControllerCommand(
            jTraj, 
            mDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            translationController, 
            strafeController, 
            thetaController, 
            mDrivetrain::setModuleStates, 
            mDrivetrain);

        addCommands(new InstantCommand(
            () -> mDrivetrain.resetOdometry(jTraj.getInitialHolonomicPose())),
                    mDriveCommand);
    }
}
