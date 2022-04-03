package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdVisionPoseEstimation extends CommandBase {

    private NAR_Drivetrain drive;
    private Limelight shooterLimelight;

    public CmdVisionPoseEstimation(NAR_Drivetrain drive, Limelight shooterLimelight) {
        this.drive = drive;
        this.shooterLimelight = shooterLimelight;

        // Don't addRequirements(drive) because we don't want to require the drivetrain
    }
    
    @Override
    public void execute() {

        if(!shooterLimelight.hasValidTarget()) {
            return;
        }

        double llDist = shooterLimelight.calculateDistToTopTarget(VisionConstants.TARGET_HEIGHT) - 11; // Distance to the front of the robot

        Pose2d visionEstimate = visionEstimatedPose(llDist, shooterLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 2), drive.getHeading());

        if (visionEstimate.getTranslation().getDistance(drive.getPose().getTranslation()) > 1.0 || translationOutOfBounds(visionEstimate.getTranslation())) {
            return;
        }

        drive.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp() - VisionConstants.LIMELIGHT_LATENCY);
    }

    private Pose2d visionEstimatedPose(double visionDist, double tx, double gyroAngle) {
        double distToHubCenter = visionDist + FieldConstants.HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyroAngle - tx);
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()), -distToHubCenter * Math.sin(thetaHub.getRadians()))
                                    .plus(FieldConstants.HUB_POSITION);
        return new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));
    }

    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FieldConstants.FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FieldConstants.FIELD_Y_LENGTH || translation.getY() < 0;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
