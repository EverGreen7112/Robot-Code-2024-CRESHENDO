package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.IntakeSubSystem;
import frc.robot.Utils.Consts;

public class IntakeWithPID extends CommandBase implements Consts {
    private Supplier<Boolean>IsPressed;
    double rpm;

    public IntakeWithPID(double rpm,Supplier<Boolean>IsPressed){
        this.rpm = rpm;
        this.IsPressed = IsPressed;
    }
    @Override
    public void initialize(){
        addRequirements(IntakeSubSystem.getInstance());
    }

    @Override
    public void execute(){
        IntakeSubSystem.getInstance().IntakeWithPID(rpm);
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
