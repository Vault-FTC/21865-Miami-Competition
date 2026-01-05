package org.firstinspires.ftc.teamcode.CommandSystem;

public abstract class CommandGroup extends Command {
    final Command[] commands;

    public CommandGroup(Command... commands) {
        this.commands = commands;
    }
}