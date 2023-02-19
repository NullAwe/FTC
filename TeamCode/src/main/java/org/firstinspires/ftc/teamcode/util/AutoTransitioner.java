//package org.firstinspires.ftc.teamcode.util;
package com.qualcomm.robotcore.eventloop.opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

public class AutoTransitioner extends Thread {

    private static final AutoTransitioner INSTANCE = new AutoTransitioner();

    private OpMode onStop;
    private String transitionTo;
    private OpModeManagerImpl opModeManager;

    private AutoTransitioner() {
        this.start();
    }

    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
        try {
            while (true) {
                synchronized (this) {
                    if (onStop != null && opModeManager.getActiveOpMode() != onStop) {
                        Thread.sleep(1000);
                        opModeManager.initActiveOpMode(transitionTo);
                        reset();
                    }
                }
                Thread.sleep(50);
            }
        } catch (InterruptedException ex) {
            Log.e("RCActivity", "AutoTransitioner shutdown, thread interrupted");
        }
    }

    private void setNewTransition(OpMode onStop, String transitionTo) {
        synchronized (this) {
            this.onStop = onStop;
            this.transitionTo = transitionTo;
            this.opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices;
        }
    }

    private void reset() {
        this.onStop = null;
        this.transitionTo = null;
        this.opModeManager = null;
    }

    public static void transitionOnStop(OpMode onStop, String transitionTo) {
        INSTANCE.setNewTransition(onStop, transitionTo);
    }
}
