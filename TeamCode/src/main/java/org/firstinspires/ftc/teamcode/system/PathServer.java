package org.firstinspires.ftc.teamcode.system;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import fi.iki.elonen.NanoHTTPD;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Config
public class PathServer extends NanoHTTPD {
    // Default port
    private static final int PORT = 8099;

    // Underlying storage exposed to Dashboard
    public static double[][] RAW_POINTS = {{0, 0, 0}};

    private static PathServer instance;

    // Constructor
    private PathServer() throws IOException {
        super(PORT);
        start(SOCKET_READ_TIMEOUT, false);
    }

    // --- Public API ---
    public static void startServer() {
        if (instance == null) {
            try {
                instance = new PathServer();
                System.out.println("PathServer started on port " + PORT);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static void stopServer() {
        if (instance != null) {
            instance.stop();
            instance = null;
            System.out.println("PathServer stopped.");
        }
    }

    public static Pose2D[] getPath() {
        Pose2D[] poses = new Pose2D[RAW_POINTS.length];
        for (int i = 0; i < RAW_POINTS.length; i++) {
            double x = RAW_POINTS[i][0];
            double y = RAW_POINTS[i][1];
            double h = RAW_POINTS[i][2];
            poses[i] = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
        }
        return poses;
    }

    // --- Internal server handling ---
    @Override
    public Response serve(IHTTPSession session) {
        try {
            // 1) Handle CORS preflight for ANY path
            if (Method.OPTIONS.equals(session.getMethod())) {
                return preflightResponse();
            }

            // 2) Handle upload
            if (Method.POST.equals(session.getMethod()) && "/points".equals(session.getUri())) {
                Map<String, String> files = new HashMap<>();
                session.parseBody(files);
                String body = files.get("postData");
                if (body == null) body = "[]";

                JSONArray arr = new JSONArray(body);
                double[][] pts = new double[arr.length()][3];
                for (int i = 0; i < arr.length(); i++) {
                    JSONArray p = arr.getJSONArray(i);
                    pts[i][0] = p.getDouble(0);
                    pts[i][1] = p.getDouble(1);
                    pts[i][2] = p.getDouble(2);
                }
                RAW_POINTS = pts;

                FtcDashboard.getInstance().updateConfig();

                return withCors(newFixedLengthResponse(Response.Status.OK, "application/json", "{\"ok\":true}"));
            }

            // 3) Not found
            return withCors(newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "not found"));

        } catch (Exception e) {
            return withCors(newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "error: " + e.getMessage()));
        }
    }

    // CORS helpers
    private Response preflightResponse() {
        Response r = newFixedLengthResponse(Response.Status.NO_CONTENT, "text/plain", "");
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true"); // important for 192.168.x.x from browsers
        r.addHeader("Access-Control-Max-Age", "600");
        return r;
    }

    private Response withCors(Response r) {
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        return r;
    }

    private Response cors(Response r) {
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Content-Type", "application/json");
        return r;
    }
}
