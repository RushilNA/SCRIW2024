package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class rumble extends Command {

    private final Joystick joystick;

    /** Constructor - Creates a new Rumble. */
    public rumble(Joystick joystick) {
        this.joystick = joystick;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        joystick.setRumble(RumbleType.kBothRumble, 1.0);  // Set rumble to max
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        joystick.setRumble(RumbleType.kBothRumble, 0);  // Stop rumble
    }

    // Command never ends on its own - it has to be interrupted.
    @Override
    public boolean isFinished() {
        return false;
    }
}
