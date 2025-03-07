package frc.robot.util;

import java.util.Locale;

public class StringUtil {
    public static String titleCase(String str) {
        if (str.length() <= 1) {
            return str.toUpperCase(Locale.ROOT);
        }

        String[] parts = str.split(" ");

        if (parts.length == 1) {
            return parts[0].substring(0, 1).toUpperCase(Locale.ROOT)
                 + parts[0].substring(1).toLowerCase(Locale.ROOT);
        }

        StringBuilder out = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            out.append(titleCase(parts[i]));
            if (i + 1 < parts.length) {
                out.append(' ');
            }
        }
        return out.toString();
    }
}
