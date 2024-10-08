package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.underthebumper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class rumbleintake extends Command {

    private final underthebumper underTheBumper;
    private final XboxController joystick;
    private final double speed;
    private final double feederSpeed;
    private boolean isRumbling;
    private Timer rumbleTimer;

    public rumbleintake(underthebumper underTheBumper, XboxController joystick, double speed, double feederSpeed) {
        this.underTheBumper = underTheBumper;
        this.joystick = joystick;
        this.speed = speed;
        this.feederSpeed = feederSpeed;
        this.isRumbling = false;
        this.rumbleTimer = new Timer();
        addRequirements(underTheBumper);
    }

    @Override
    public void initialize() {
        rumbleTimer.stop();
        rumbleTimer.reset();
        isRumbling=false;
    }

    @Override
    public void execute() {
        if(underTheBumper.beamborken()== false){underTheBumper.set(speed, feederSpeed);}
       if (underTheBumper.beamborken()==true) {
        underTheBumper.set(0, 0 );
        
       } else {
        
       }

        if (underTheBumper.beamborken() && !isRumbling) {
            isRumbling = true;
            rumbleTimer.start();
            joystick.setRumble(RumbleType.kBothRumble, 10);  // Start rumble
        }

        if (isRumbling && rumbleTimer.hasElapsed(3.0)) {
            joystick.setRumble(RumbleType.kBothRumble, 0);  // Stop rumble after 3 seconds
            isRumbling = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        underTheBumper.set(0,0);
        joystick.setRumble(RumbleType.kBothRumble, 0);  // Ensure rumble is stopped
        rumbleTimer.stop();
        rumbleTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
