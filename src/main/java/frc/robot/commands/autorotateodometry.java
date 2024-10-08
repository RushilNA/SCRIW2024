package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;

public class autorotateodometry extends Command {
    private SwerveSubsystem s_Swerve; 

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private PIDController turnController;

    private Pose2d targetPose;

    public autorotateodometry(SwerveSubsystem s_Swerve, Pose2d targetPose, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        this.targetPose = targetPose;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        // Initialize the PID controller for rotation
        turnController = new PIDController(0.01, 0.0001, 0.000005);
        turnController.enableContinuousInput(-180, 180);

        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        // Get current robot pose
        Pose2d currentPose = s_Swerve.getPose();

        // Calculate delta translation from current pose to target pose
        Translation2d deltaTranslation = targetPose.getTranslation().minus(currentPose.getTranslation());

        // Calculate target rotation based on delta translation
        Rotation2d targetRotation = deltaTranslation.getAngle();

        // Calculate current rotation
        Rotation2d currentRotation = currentPose.getRotation();

        // Calculate rotation needed to face the target
        Rotation2d rotateToTarget = targetRotation.minus(currentRotation);

        // Get rotation speed from PID controller
        double rotationSpeed = turnController.calculate(currentRotation.getDegrees(), targetRotation.getDegrees());

        // Apply deadband to translation and strafe values
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND);

        // Drive the robot with the calculated translation and rotation speeds
        s_Swerve.drive1(
            new Translation2d(translationVal, strafeVal).times(14),
            rotationSpeed,
            robotCentricSup.getAsBoolean()
        );
    }
}
