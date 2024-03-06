package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Consts.ShooterValues;

public class NotAmpSIdeAuto extends Command implements Consts{
    @Override
    public void initialize() {
        new SequentialCommandGroup(
        new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(114);}),
                                     new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);}))
                ,new WaitCommand(2)
                ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED); }) 
                ,new WaitCommand(0.5)
                ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(0); 
                                          Shooter.getInstance().setShootSpeed(0);
                                          Shooter.getInstance().stopRollers();
                                          Intake.getInstance().stopMotor();})
                ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, -0.75), false);})                         
                ).schedule();;
    }
}
