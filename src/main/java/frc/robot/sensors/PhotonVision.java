package frc.robot.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.PhotonVisionConstants;

/** PhotonVision subsystem for handling multiple cameras and pose estimation */
public class PhotonVision {
    private static final PhotonCamera m_camera0 = new PhotonCamera(PhotonVisionConstants.kCamera0Name);
    // private static final PhotonCamera m_camera1 = new PhotonCamera(PhotonVisionConstants.kCamera1Name);
    // private static final PhotonCamera m_camera2 = new PhotonCamera(PhotonVisionConstants.kCamera2Name);

    private static final PhotonCamera[] m_cameras = {
        m_camera0,
        // m_camera1,
        // m_camera2
    };

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp);
    }

    private static class PhotonVisionSingleCameraPoseEstimator {
        private final PhotonCamera m_camera;
        private final PhotonPoseEstimator m_poseEstimator;
        private final Supplier<VisionSystemSim> m_visionSimSupplier;
        private final EstimateConsumer estConsumer;
        private PhotonPipelineResult cachedLatestResult; // Cached latest result for use in getLatestAngleToTarget

        public PhotonVisionSingleCameraPoseEstimator(Transform3d robotToCam, PhotonCamera camera, Supplier<VisionSystemSim> visionSimSupplier, EstimateConsumer estimateConsumer) {
            estConsumer = estimateConsumer;
            m_poseEstimator = new PhotonPoseEstimator(PhotonVisionConstants.kApriltagFieldLayout, robotToCam);
            m_camera = camera;
            m_visionSimSupplier = visionSimSupplier;
        }

        private void update() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var result : m_camera.getAllUnreadResults()) {
                cachedLatestResult = result; // Updates the cached latest result with the most recent unread result
                visionEst = m_poseEstimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = m_poseEstimator.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEst, result.getTargets());
                
                if (Robot.isSimulation()) {
                    visionEst.ifPresentOrElse(
                        est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
                }

                visionEst.ifPresent(
                    est -> {
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    }
                );
            }
        }

        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (!estimatedPose.isEmpty()) {
                var estStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                for (var tgt: targets) {
                    var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    numTags++;
                    avgDist += tagPose.get()
                                      .toPose2d()
                                      .getTranslation()
                                      .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags != 0) {
                    avgDist /= numTags;
                    if (numTags > 1) estStdDevs = PhotonVisionConstants.kMultiTagStdDevs;
                    if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
            }
        }

        public Field2d getSimDebugField() {
            if (!Robot.isSimulation()) return null;
            return m_visionSimSupplier.get().getDebugField();
        }

        /**
         * Returns the angle in degrees to the target from the latest result of this camera (with left being the positive direction).
         * 
         * Returns 0 if there is no target detected in the latest result.
         */
        public double getLatestAngleToTarget() {
            var result = cachedLatestResult;

            if (result != null && result.hasTargets()) { // Checks if there is a valid cached result with targets
                //return result.getBestTarget().getYaw(); // Returns the yaw of the target in degrees, with left being the positive direction.

                PhotonTrackedTarget highValueTarget = getHighValueTarget(result); // Gets the highest value target from the result

                if (highValueTarget != null) { // Checks if a high value target was found
                    return highValueTarget.getYaw(); // Returns the yaw of the high value target in degrees, with left being the positive direction.
                }
            }
            return 0;
        }

         /**
         * Returns The area of the target as a percentage of the camera image from the latest result of this camera
         * 
         * Returns 0 if there is no target detected in the latest result.
         */
        public double getLatestAreaOfTarget() {
            var result = cachedLatestResult;

            if (result != null && result.hasTargets()) { // Checks if there is a valid cached result with targets
                //return result.getBestTarget().getYaw(); // Returns the yaw of the target in degrees, with left being the positive direction.

                PhotonTrackedTarget highValueTarget = getHighValueTarget(result); // Gets the highest value target from the result

                if (highValueTarget != null) { // Checks if a high value target was found
                    return highValueTarget.getArea(); // Returns the yaw of the high value target in degrees, with left being the positive direction.
                }
            }
            return 0;
        }

        public int getLatestTargetId() {
            var result = cachedLatestResult;

            if (result != null && result.hasTargets()) {
                PhotonTrackedTarget highValueTarget = getHighValueTarget(result);
                if (highValueTarget != null) {
                    return highValueTarget.getFiducialId();
                }
            }
            return -1;
        }

        /**
         * Returns the highest value target from the given result, with the order of importance being:
         * 1. Hubs, directly in front of the alliance stations
         * 2. Hubs, from the sides of the field 
         * @param result the result to search through for targets
         * @return the highest value target, or null if no high value target is found
         */
        public PhotonTrackedTarget getHighValueTarget(PhotonPipelineResult result) {

		if (result.hasTargets()) {

			List<PhotonTrackedTarget> targets = result.getTargets();

			for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_HUB_FRONT_RIGHT_SUPER_TARGET 
					|| targetId == AprilTags.BLUE_HUB_FRONT_RIGHT_SUPER_TARGET)
				{
					return target; // SUPER high value target found - more important than high value only
				}
			}

			for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_HUB_RIGHT_RIGHT_TARGET
					|| targetId == AprilTags.BLUE_HUB_RIGHT_RIGHT_TARGET)
				{
					return target; // high value target found
				}
			}

            for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_HUB_LEFT_LEFT_TARGET
					|| targetId == AprilTags.BLUE_HUB_LEFT_LEFT_TARGET)
				{
					return target; // high value target found
				}
			}
		}

		return null; // no high value target found
	}
    }

    
    public static class PhotonVisionEstimationSubsystem extends SubsystemBase {
        private ArrayList<PhotonVisionSingleCameraPoseEstimator> m_cameraEstimationsSubs;
        private ArrayList<PhotonCameraSim> m_cameraSims;
        private VisionSystemSim m_visionSim;
        
        public PhotonVisionEstimationSubsystem(EstimateConsumer estConsumer) {
            m_cameraEstimationsSubs = new ArrayList<PhotonVisionSingleCameraPoseEstimator>();
            for (int i = 0; i < PhotonVisionConstants.kRobotToCams.length; i++) {
                PhotonVisionSingleCameraPoseEstimator cam = new PhotonVisionSingleCameraPoseEstimator(PhotonVisionConstants.kRobotToCams[i], m_cameras[i], () -> m_visionSim, estConsumer);
                m_cameraEstimationsSubs.add(cam);
            }

            if (Robot.isSimulation()) {
                m_visionSim = new VisionSystemSim("main");
                m_visionSim.addAprilTags(PhotonVisionConstants.kApriltagFieldLayout);

                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
                cameraProp.setCalibError(0.35, 0.10);
                cameraProp.setFPS(15);
                cameraProp.setAvgLatencyMs(50);
                cameraProp.setLatencyStdDevMs(15);

                m_cameraSims = new ArrayList<PhotonCameraSim>();
                for (var cam : m_cameras) {
                    PhotonCameraSim sim = new PhotonCameraSim(cam, cameraProp);
                    m_cameraSims.add(sim);
                }

                for (int i = 0; i < m_cameraSims.size(); i++) {
                    m_visionSim.addCamera(m_cameraSims.get(i), PhotonVisionConstants.kRobotToCams[i]);
                }

                for (var sim : m_cameraSims) {
                    sim.enableDrawWireframe(true);
                }
            }

            this.setDefaultCommand(new PhotonVisionEstimationCommand(this));
        }
        
        public void updateEstimations() {
            for (var cam : m_cameraEstimationsSubs) {
                cam.update();
            }
        }

        /**
         * Returns the angle in degrees to the target from the latest result of the first camera (with left being the positive direction).
         * 
         * Returns 0 if there is no target detected in the latest result.
         */
        public double getLatestAngleToTarget() {
            if (!m_cameraEstimationsSubs.isEmpty()) { // Checks if there is at least one camera estimation subsystem
                return m_cameraEstimationsSubs.get(0).getLatestAngleToTarget();
            }
            return 0;
        }

        /**
         * Returns The area of the target as a percentage of the camera from the latest result of the first camera
         * 
         * Returns 0 if there is no target detected in the latest result.
         */
        public double getLatestAreaOfTarget() {
            if (!m_cameraEstimationsSubs.isEmpty()) { // Checks if there is at least one camera estimation subsystem
                return m_cameraEstimationsSubs.get(0).getLatestAreaOfTarget();
            }
            return 0;
        }

        public int getLatestTargetId() {
            if (!m_cameraEstimationsSubs.isEmpty()) {
                return m_cameraEstimationsSubs.get(0).getLatestTargetId();
            }
            return -1;
        }
    }

    private static class PhotonVisionEstimationCommand extends Command {
        private PhotonVisionEstimationSubsystem m_photonVision;
        public PhotonVisionEstimationCommand(PhotonVisionEstimationSubsystem photonVision) {
            addRequirements(photonVision);
            m_photonVision = photonVision;
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void execute() {
            m_photonVision.updateEstimations();
        }
    }
}
