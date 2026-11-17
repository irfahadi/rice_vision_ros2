package ej.microui;
/** Stub: di board, Runnable dijalankan di thread UI. Di sim dijalankan langsung (serial). */
public final class MicroUI {
    private MicroUI() {}
    public static synchronized void callSerially(Runnable r) { r.run(); }
}
