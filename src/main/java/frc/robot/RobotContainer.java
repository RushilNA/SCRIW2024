// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.autointake;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;


// import frc.robot.subsystems.intakesub;
import java.io.File;
import java.nio.file.Path;
import java.util.logging.Logger;

import javax.naming.PartialResultException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

// import frc.robot.subsystems.Limelight;
// // import frc.robot.subsystems.PhotonVision;
// import frc.robot.subsystems.PhotonVision;


// // import frc.robot.subsystems.PhotonVision;

// import frc.robot.subsystems.limek1;
// import frc.robot.subsystems.lookuptable.lookuptable;
import frc.robot.commands.LookUpShotoot;
import frc.robot.commands.autonotepickup;
import frc.robot.commands.autorotate;
import frc.robot.commands.autorotateodometry;
import frc.robot.commands.rumble;
import frc.robot.commands.rumbleintake;
import frc.robot.subsystems.Boxpiv;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.cam2photon;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.projectiles;
// import frc.robot.subsystems.outake;
import frc.robot.subsystems.underthebumper;
// import frc.robot.subsystems.PhotonVision;

import frc.robot.subsystems.PhotonVision;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  //  private lookuptable m_Visionlookuptable = new lookuptable();


  // The robot's subsystems and c ommands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // private final Limelight limelight = new Limelight();



  private final Boxpiv boxpivsub = new Boxpiv();
//  private final Climb climbsub = new Climb();
  private final projectiles projectilesub = new projectiles();
  private final PhotonVision lime = new PhotonVision(drivebase);
  private final cam2photon cam2 = new cam2photon();
  private final underthebumper newintake = new underthebumper();


  
  // private final PhotonVision Vision = new PhotonVision(drivebase);


                                       
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
   private final SendableChooser<Command> autoChooser2 = new SendableChooser<Command>();




  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driver = new XboxController(0);


  // private final JoystickButton driver_limelightButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


  private final JoystickButton climbpur = new JoystickButton(driver, XboxController.Button.kRightStick.value);//p2
  private final JoystickButton subwoofer = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton podium = new JoystickButton(operator, XboxController.Button.kA.value);

  //private final JoystickButton intakehmanplayerButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
   private final JoystickButton AMPButton = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
   
      private final JoystickButton Outake = new JoystickButton(operator, XboxController.Button.kBack.value);
      private final JoystickButton autointake = new JoystickButton(driver, XboxController.Button.kX.value);
            private final JoystickButton autoroate = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton A = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton X = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton B = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton Y = new JoystickButton(driver, XboxController.Button.kX.value); 
    // private final JoystickButton Deadopen = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton C = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton D = new JoystickButton(driver, XboxController.Button.kX.value);
    // private  Pose2d AP = new Pose2d();
    // private  Pose2d XP = new Pose2d();

  


  private final POVButton UnjamButton= new POVButton(operator, 180);
    private final POVButton feaderf= new POVButton(operator, 0);
      private final POVButton outakeunjam= new POVButton(operator, 90);
    private final POVButton outakeunjam1= new POVButton(driver, 270);
  private final JoystickButton climbr = new JoystickButton(driver, XboxController.Button.kBack.value);//p3
   private final JoystickButton climbpul = new JoystickButton(driver, XboxController.Button.kStart.value);//p4
    private final JoystickButton climbpurd = new JoystickButton(driver, XboxController.Button.kLeftStick.value);//p1
        private final Trigger rightTrigger = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5); // Adjust threshold as needed Transfer
              private final Trigger lefTrigger = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5); // Adjust threshold as needed Auto shooter piv 
            



      private final JoystickButton subwooferpiv = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
          private final JoystickButton humanintake = new JoystickButton(driver, XboxController.Button.kB.value);















  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()



  {
    // hub.setSwitchableChannel(true);
    // m_vision.useLimelight(true);
    // m_vision.setAlliance(Alliance.Blue);
    // m_vision.trustLL(true);
     // Correct pose estimate with vision measurements
     Rotation2d cRotation2d = drivebase.getPose().getRotation();

    

      

    
    boxpivsub.encoder();
    // projectilesub.shuffleboard();
    // drivebase.llreset();




    NamedCommands.registerCommand("runintake", newintake.intakeandfeeder(0.7, -0.7));
    NamedCommands.registerCommand("pullback", newintake.intakeandfeederauto(0,0.4 ,.4));
        NamedCommands.registerCommand("pullbacko", projectilesub.Outtake(-1));
        

    NamedCommands.registerCommand("runintake2", newintake.intakeandfeederauto(0.8,0.80,.5));
    NamedCommands.registerCommand("boxpivclose",boxpivsub.boxpivcmdAU(-9.8736328125));
    NamedCommands.registerCommand("boxpivfar",boxpivsub.boxpivcmdAU1(-4.2));
    NamedCommands.registerCommand("boxpivfar1",boxpivsub.boxpivcmdAU1(-4.0));
    NamedCommands.registerCommand("boxpivfar2",boxpivsub.boxpivcmdAU1(-6.0));
    NamedCommands.registerCommand("shoot",projectilesub.OtakeAU(.7));

    
    //Make Multiple of the below shoot speakers because you need to manually find the setpoints and speed for each angle
    
    //autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("hi", new SequentialCommandGroup());
   
        autoChooser.addOption("2 piece stage ", new PathPlannerAuto("2 Piece Stage"));
        autoChooser.addOption("3 Piece AMP Side", new PathPlannerAuto("3 Piece AMP Side"));
        autoChooser.addOption("3 Piece Stage Side", new PathPlannerAuto("Copy of 3 Piece AMP Side"));
            autoChooser.addOption("2 piece amp", new PathPlannerAuto("Ashrith"));
             autoChooser.addOption("2 piece center", new PathPlannerAuto("Copy of 2 Piece Center Real hi"));
             autoChooser.addOption("Center Test", new PathPlannerAuto("Copy of 3 Piece PT1"));
             autoChooser.addOption("Bazinga!", new SequentialCommandGroup(new PathPlannerAuto("Copy of 3 Piece PT1"),new PathPlannerAuto("3 Piece PT2"),new PathPlannerAuto("3 Piece PT3")));

    //   PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Straight");
    // Command amp1 = new SequentialCommandGroup(AutoBuilder.followPath(exampleChoreoTraj));
    // autoChooser2.addOption("Straight", amp1);


    drivebase.odometrygetshuffleboard();

    configureBindings();
     SmartDashboard.putData("Auto chooser",autoChooser);

    //  SmartDashboard.putData("Auto Chooser SCrap", autoChooser2);
   
    // projectilesub.shuffleboard();
  

    teleop closedFieldRelOperator = new teleop(
      drivebase,
      () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),

      () -> -driver.getRightX(), () -> true);
      

    drivebase.setDefaultCommand(closedFieldRelOperator);
   
    



  }



  private void configureBindings()
  {
    
    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
    //   XP = new Pose2d(1.8,8.06,new Rotation2d(0));

       
    // }

Pose2d targetpose = new Pose2d(0,0,Rotation2d.fromRadians(90));
  


    new JoystickButton(driver, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
  

    // 
    










    Command autoRotateCommand = new autorotateodometry(
      drivebase,
      targetpose,
      () -> driver.getLeftY(), // Translation supplier
      () -> driver.getLeftX(), // Strafe supplier
      () -> driver.getRightX(), // Rotation supplier
      () -> false // Robot centric supplier (assuming trigger for this example)
  );

  // Bind the command to the button




lefTrigger.whileTrue(new LookUpShotoot(boxpivsub, projectilesub,() ->drivebase.calcDistToSpeaker(), newintake)).whileFalse(boxpivsub.boxpivcmdTO(0));

rightTrigger.whileTrue(new ParallelCommandGroup(newintake.intakeandfeeder1(0.6,-0.75))).whileFalse(new ParallelCommandGroup(newintake.intakeandfeeder1(0, 0)));

podium.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-3.43359375),projectilesub.Outtake(0.65))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));
subwooferpiv.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-8.1),projectilesub.Outtake(0.65))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));

humanintake.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-8),projectilesub.Outtake(-0.65))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));

//  AMPButton.whileTrue(boxpivsub.AMPStopper(-162.72303771972656)).whileFalse(boxpivsub.AMPStopper(0));
  AMPButton.whileTrue(autoRotateCommand); 
//  AMPButton.whileTrue(
//     new ParallelCommandGroup(
        
//       new SequentialCommandGroup(new ParallelCommandGroup(boxpivsub.boxpivcmdTOamp(-19.45), boxpivsub.AMPStopper(-162.72303771972656)),new ParallelCommandGroup(projectilesub.Outtake(0.20),boxpivsub.boxpivcmdTO(-19.45),boxpivsub.AMPStopper(-162.72303771972656)))
//     ) ).whileFalse(
//    new ParallelCommandGroup(
//        projectilesub.Outtake(0),
//        boxpivsub.boxpivcmdTO(0),
//         boxpivsub.AMPStopper(0)
//     ));
     AMPButton.whileTrue(
    new ParallelCommandGroup(
        
      new SequentialCommandGroup(new ParallelCommandGroup( boxpivsub.AMPStopper(-162.72303771972656)),new ParallelCommandGroup(boxpivsub.AMPStopper(-162.72303771972656)))
    ) ).whileFalse(
   new ParallelCommandGroup(
       
        boxpivsub.AMPStopper(0)
    ));
// AMPButton.whileTrue(
//     new ParallelCommandGroup(
        
//         new SequentialCommandGroup(new ParallelCommandGroup(boxpivsub.boxpivcmdTOamp(-19.45)),new ParallelCommandGroup(projectilesub.Outtake(0.25),boxpivsub.boxpivcmdTO(-19.45)))
//     )
// ).whileFalse(
//     new ParallelCommandGroup(
//         projectilesub.Outtake(0),
//         boxpivsub.boxpivcmdTO(0)
      
//     )
// );
// AMPButton.whileTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(boxpivsub.boxpivcmdTOamp(-13), projectilesub.OuttakeAmp(.20)), new ParallelDeadlineGroup(boxpivsub.boxpivcmdTOamp(-20.1),newintake.autoampshit(-0.8), projectilesub.OuttakeAmp(.19)))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTOamp(0),newintake.intakeandfeederauto(0,0,0)));

Outake.whileTrue(projectilesub.Outtake(.8)).whileFalse(projectilesub.Outtake(0));



// p21.whileTrue(projectilesub.Outtake(0.9)).whileFalse(projectilesub.Outtake(0));
// p22.whileTrue(newintake.intakeandfeeder(0.65,0.75));  
// intakeButton.whileTrue(
// (
//   new ParallelCommandGroup(new rumbleintake(newintake, driver, 0.65, 0.75))),new autointake(
//     newintake,
//       cam2,
//       drivebase,
//       () -> driver.getLeftY(),
//       () -> driver.getLeftX(), 

//       () -> driver.getRightX(), () -> false));

  // intakeButton.whileTrue(
  //   new ParallelCommandGroup(
  //     new rumbleintake(newintake, driver, 0.8, -0.75),
  //     new autonotepickup(cam2, drivebase,  () -> driver.getLeftY(), () -> driver.getLeftX(),  () -> driver.getRightX(), () -> false)



  //   )
    



  // ).whileFalse(newintake.intakeandfeeder(0, 0));
  intakeButton.whileTrue(
    new SequentialCommandGroup(
      new rumbleintake(newintake, driver, 0.9, -0.8)
     



    )
    



  ).whileFalse(newintake.intakeandfeeder(0, 0));

   
// intakeButton.whileTrue(newintake.intakeandfeeder(0.65, 0.75));





outakeunjam1.whileTrue(new ParallelCommandGroup( projectilesub.Outtake(-0.5),newintake.intakeandfeeder(-0.9,0.9))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),newintake.intakeandfeeder(0, 0)));




// autointake.whileTrue(new autointake(
//     newintake,
//       cam2,
//       drivebase,
//       () -> driver.getLeftY(),
//       () -> driver.getLeftX(), 

//       () -> driver.getRightX(), () -> false)).whileFalse(newintake.intakeandfeeder(0, 0));  

  
  
		
    // autoroate.whileTrue(new autointake(newintake,cam2,drivebase,() -> driver.getLeftY(),
    //   () -> driver.getLeftX(), 

    //   () -> driver.getRightX(), () -> false));  

    // Auto Aim Speaker 
    // driver_limelightButton.whileTrue(
      
    // new LookUpShotoot(boxpivsub, projectilesub,() ->drivebase.calcDistToSpeaker(), newintake))
    //   ;
      
        
      
      
    // climbr.toggleOnTrue(climbsub.solenoidCommand(true,-0.6)).toggleOnFalse(climbsub.solenoidCommand(false,0));
    // climbpul.whileTrue(climbsub.ClimbCmd1(0.3)).whileFalse(climbsub.ClimbCmd1(0));//up?
    // climbpur.whileTrue(climbsub.ClimbCmd2(0.3)).whileFalse(climbsub.ClimbCmd2(0));//down?
    
    // climbpurd.whileTrue(climbsub.climbcmd(0.6)).whileFalse(climbsub.climbcmd(0));
 

  
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    

    return autoChooser.getSelected();
    // return new SequentialCommandGroup();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }
  public void camera(){
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}