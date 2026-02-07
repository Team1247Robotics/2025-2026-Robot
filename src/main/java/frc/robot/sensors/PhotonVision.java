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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PhotonVisionConstants;

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

        public PhotonVisionSingleCameraPoseEstimator(Transform3d robotToCam, PhotonCamera camera, Supplier<VisionSystemSim> visionSimSupplier, EstimateConsumer estimateConsumer) {
            estConsumer = estimateConsumer;
            m_poseEstimator = new PhotonPoseEstimator(PhotonVisionConstants.kApriltagFieldLayout, robotToCam);
            m_camera = camera;
            m_visionSimSupplier = visionSimSupplier;
        }

        private void update() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var result : m_camera.getAllUnreadResults()) {
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
