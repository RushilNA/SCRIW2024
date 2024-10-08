package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.cam2photon;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;


import static edu.wpi.first.units.Units.*;



public class autonotepickup extends Command {
    private cam2photon s_vision;
    private SwerveSubsystem s_Swerve; 

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double storedTranslationVal;
    private double storedStrafeVal;
    private double storedRotationVal;
    private boolean targetLocked = false; // Flag to indicate if a target has been found
    private SwerveController controller;

    public autonotepickup(cam2photon photonvision, SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        s_vision = photonvision;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        this.controller = s_Swerve.getSwerveController();

        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), OperatorConstants.RIGHT_X_DEADBAND);

        PIDController turnController = new PIDController(0.04, 0.0001, 0.000005);
        turnController.enableContinuousInput(-180, 180);

        var result = s_vision.getLatestResult();
        
        if(result.hasTargets()) {
            double rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

            // If the target is found for the first time, store the current joystick inputs
            if (!targetLocked) {
                storedTranslationVal = translationVal;
                storedStrafeVal = strafeVal;
                storedRotationVal = rotationVal;
                targetLocked = true;
            }

            // Use the stored joystick values to keep the robot moving in the same direction
            s_Swerve.drive1(
                new Translation2d(storedTranslationVal, storedStrafeVal).times(14),
                -rotationSpeed * 1.5,
                robotCentricSup.getAsBoolean()
            );
        } else {
            // If no target is detected, allow the joystick to control the robot as usual
            targetLocked = false;

            s_Swerve.drive1(
                new Translation2d(translationVal, strafeVal).times(14),
                rotationVal * controller.config.maxAngularVelocity / 1.5,
                robotCentricSup.getAsBoolean()
            );
        }
    }
}
