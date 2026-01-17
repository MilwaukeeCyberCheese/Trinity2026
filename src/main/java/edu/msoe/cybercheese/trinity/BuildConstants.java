package edu.msoe.cybercheese.trinity;

import org.jspecify.annotations.Nullable;

import java.io.IOException;
import java.net.JarURLConnection;
import java.time.Instant;
import java.util.Objects;
import java.util.jar.Manifest;

public final class BuildConstants {
    private static final @Nullable Manifest MANIFEST = readManifest();

    private static @Nullable Manifest readManifest() {
        final var url = BuildConstants.class.getResource(BuildConstants.class.getSimpleName() + ".class");

        if (url == null || !Objects.equals(url.getProtocol(), "jar")) {
            return null;
        }

        try {
            final var conn = (JarURLConnection) url.openConnection();

            return conn.getManifest();
        } catch (IOException e) {
            return null;
        }
    }

    private static String manifestKey(String property) {
        if (MANIFEST == null) return "UNKNOWN";

        final var value = MANIFEST.getMainAttributes().getValue(property);
        if (value == null) return "UNKNOWN";

        return value;
    }

    public static final long BUILD_TIME_MILLIS = Long.parseLong(manifestKey("Build-Timestamp"));
    public static final Instant BUILD_TIME = Instant.ofEpochMilli(BUILD_TIME_MILLIS);
    public static final String TITLE = BuildConstants.class.getPackage().getImplementationTitle();
    public static final String VERSION = BuildConstants.class.getPackage().getImplementationVersion();

    public static final String GIT_COMMIT = manifestKey("Git-Commit");
    public static final String GIT_BRANCH = manifestKey("Git-Branch");

    private BuildConstants() {}
}
