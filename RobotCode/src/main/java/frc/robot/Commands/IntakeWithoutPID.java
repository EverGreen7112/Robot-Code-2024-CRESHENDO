package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.IntakeSubSystem;
import frc.robot.Utils.Consts;

public class IntakeWithoutPID extends CommandBase implements Consts {
    private Supplier<Boolean>IsPressed;
    double speed;

    public IntakeWithoutPID(double speed,Supplier<Boolean>IsPressed){
        this.speed = speed;
        this.IsPressed = IsPressed;
    }
    @Override
    public void initialize(){
        addRequirements(IntakeSubSystem.getInstance());
    }

    @Override
    public void execute(){
        IntakeSubSystem.getInstance().IntakeWithoutPID(speed);
    }
    @Override
    public boolean isFinished(){
        return IsPressed.get();
    }

    @Override
    public void end(boolean interrupted){
        IntakeSubSystem.getInstance().m_intakeMotor.stopMotor();
    }
}