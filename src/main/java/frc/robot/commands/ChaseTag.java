package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class ChaseTag extends CommandBase {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2;  
    private static final Transform3d TAG_TO_GOAL =  
        new Transform3d(
            new Translation3d(1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );
    
    private final PhotonCamera m_PhotonCamera;
    private final SwerveDrivetrain m_Drivetrain;

    private final ProfiledPIDController xController = new ProfiledPIDController(AutonConstants.kPXController, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(AutonConstants.kPYController, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(AutonConstants.kPThetaController, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public ChaseTag(PhotonCamera cam, SwerveDrivetrain dt) {
        m_PhotonCamera = cam;
        m_Drivetrain = dt;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_Drivetrain);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        var robotPose = m_Drivetrain.getPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    } 
    
    @Override
    public void execute() {
      var robotPose2d = m_Drivetrain.getPose();
      var robotPose = 
          new Pose3d(
              robotPose2d.getX(),
              robotPose2d.getY(),
              0.0, 
              new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
      
      var photonRes = m_PhotonCamera.getLatestResult();
      if (photonRes.hasTargets()) {
        // Find the tag we want to chase
        var targetOpt = photonRes.getTargets().stream()
            .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
            .findFirst();
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          // This is new target data, so recalculate the goal
          lastTarget = target;
          
          // Transform the robot's pose to find the camera's pose
          var cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAM);
  
          // Trasnform the camera's pose to the target's pose
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          
          // Transform the tag's pose to set our goal
          var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
  
          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
      
      if (lastTarget == null) {
        // No target has been visible
        m_Drivetrain.stopSwerve();
      } else {
        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
          xSpeed = 0;
        }
  
        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
          ySpeed = 0;
        }
  
        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        if (omegaController.atGoal()) {
          omegaSpeed = 0;
        }
  
        m_Drivetrain.setChassisSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_Drivetrain.stopSwerve();
    }
  
}
