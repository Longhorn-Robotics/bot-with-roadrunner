package org.firstinspires.ftc.teamcode.GIGACHAD.utils;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

// Actually don't use this
// The new model will need something that likely isn't multithreaded
//  
public class Timeout {
    private static final ScheduledExecutorService scheduler =
            Executors.newScheduledThreadPool(1);

    public static ScheduledFuture<?> setTimeout(int delay, Runnable runnable) {
        return scheduler.schedule(runnable, delay, TimeUnit.MILLISECONDS);
    }

    public static void cancelTimeout(ScheduledFuture<?> future) {
        if (future != null) {
            future.cancel(true);
        }
    }

    public static void shutdown() {
        scheduler.shutdown();
    }
}
