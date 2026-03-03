package edu.msoe.cybercheese.trinity.util;

public record FFConstants(double ks, double kv) {

    public static final FFConstants INVALID = new FFConstants(-1, -1);
}
