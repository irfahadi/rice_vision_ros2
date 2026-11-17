package ej.bon;
public final class Util {
    private Util() {}
    public static long platformTimeMillis() { return System.nanoTime() / 1_000_000L; }
    public static long currentTimeMillis() { return System.currentTimeMillis(); }
}
