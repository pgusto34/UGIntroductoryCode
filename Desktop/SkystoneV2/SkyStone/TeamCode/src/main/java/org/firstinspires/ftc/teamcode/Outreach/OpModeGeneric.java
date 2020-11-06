package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public abstract class OpModeGeneric extends OpMode {

    protected Map<String, Future<?>> tasks = new HashMap<>();

    // Thread Pool
    protected ThreadPoolExecutor executor =
            (ThreadPoolExecutor) Executors.newFixedThreadPool(4);

    protected void submitFunc(Runnable func, String taskName) {
        Future<?> out = executor.submit(func);
        tasks.put(taskName, out);
    }

    // Timing thread pool
    private ScheduledThreadPoolExecutor timer = new ScheduledThreadPoolExecutor(2);
    protected int period = 1;
    protected TimeUnit timeUnit = TimeUnit.MILLISECONDS;


    // Abstract Functions
    protected void initMotors() {}
    protected void initServos() {}
    protected void initSensors() {}
    protected void initLong() {}
    protected void initOther() {}


    /**
     * Calls the initialization functions declared by the child classes. initMotors, initServos,
     * initSensors, and initOther all run sequentially (in that order), whereas initLong runs in a
     * separate thread so as to prevent init blocking (long running functions could include things
     * like Vuforia initialization, which takes upwards of 7 seconds to complete.
     */
    public void init() {
        initMotors();
        initServos();
        initSensors();
        initOther();

        Runnable runLong = new Runnable(){
            public void run() {
                initLong();
            }
        };

        submitFunc(runLong, "initLong");
    }

    protected void newStart() {}

    public void start() {
        newStart();

        timer.scheduleAtFixedRate(new Runnable() {
            public void run() {
                timerLoop();
            }
        }, 0, period, timeUnit);
    }

    protected abstract void timerLoop();

    public void loop() {}

    public void stop() {
        executor.shutdownNow();
        timer.shutdownNow();
    }
    
}
