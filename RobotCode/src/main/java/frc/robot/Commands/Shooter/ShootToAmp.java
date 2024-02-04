package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utils.Consts.ShooterValues;

public class ShootToAmp extends Command{
    @Override
    public void initialize() {
        Shooter.getInstance().shootNote(ShooterValues.AMP_SHOOT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopRollers();
    }
}
