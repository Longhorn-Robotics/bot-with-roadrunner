package org.firstinspires.ftc.teamcode.GIGACHAD.utils;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

// TODO: Better Macro Creation
// Sort of like:
// Macro m = new Macro().action(delay, {})
//                      .action(delay, {});
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
