package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;

public class CmdReverseIntake extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;

    public CmdReverseIntake(Intake intake, Hopper hopper) {
        m_intake = intake;
        m_hopper = hopper;

        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.runIntakeBack();
        m_hopper.reverseHopper();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_intake.retractIntake();
    }

    public boolean isFinished() {
        return false;
    }
}
