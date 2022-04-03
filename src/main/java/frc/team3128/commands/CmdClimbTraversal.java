package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.Constants.ClimberConstants;
import frc.team3128.subsystems.Climber;

public class CmdClimbTraversal extends SequentialCommandGroup {

    public CmdClimbTraversal(Climber m_climber) {
        addCommands(

                // new CmdExtendIntake(m_intake),

                // Climber is manually fully retracted on Mid Bar
                new InstantCommand(() -> m_climber.retractPiston()),
                new CmdClimbEncoder(m_climber, -350),
                new WaitCommand(0.5),
                // elev extend a wee bit
                new CmdClimbEncoder(
                        m_climber,
                        m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),
                new WaitCommand(0.5),

                // piston extend
                new InstantCommand(() -> m_climber.extendPiston()),
                new WaitCommand(0.25),

                // elev extend
                new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
                new WaitCommand(0.25),

                // piston retract
                new InstantCommand(() -> m_climber.retractPiston()),

                // elev retract
                new CmdClimbEncoder(m_climber, -350),

                // Climber is manually fully retracted on High Bar

                new WaitCommand(0.25),

                // elev extend
                new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
                new WaitCommand(0.75),

                // piston extend
                new InstantCommand(() -> m_climber.extendPiston()),
                new WaitCommand(0.5),

                // maybe comment this and everything under it and have mak kenna manual the rest
                new CmdClimbEncoder(m_climber, 1000),
                new WaitCommand(0.5),

                // new ParallelCommandGroup(
                //     new CmdClimbEncoder(m_climber, 2500),
                //     new SequentialCommandGroup(new RunCommand(() -> {}).withInterrupt(() ->
                // m_climber.getAvgCurrent() > 20),
                //                                 new InstantCommand(() ->
                // m_climber.retractPiston()))
                // )

                // piston retract
                new InstantCommand(() -> m_climber.retractPiston())
                // new WaitCommand(0.5),

                // elev retract
                // new CmdClimbEncoder(m_climber, 3000) // Aaron number
                );
    }
}
