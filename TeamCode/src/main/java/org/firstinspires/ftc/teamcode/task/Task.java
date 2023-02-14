package org.firstinspires.ftc.teamcode.task;

public interface Task {

    boolean perform();

    default void cancel(){}

}
