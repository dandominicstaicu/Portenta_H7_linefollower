#include "WebServerHandler.h"

void handleWeb() {
    WiFiClient client = server.accept();
    if (!client) return;

    String req = client.readStringUntil('\r');
    client.readStringUntil('\n');

    if (req.startsWith("GET /update?")) {
        int p = req.indexOf('?') + 1;
        int e = req.indexOf(' ', p);
        String qs = req.substring(p, e);

        unsigned int i = 0;
        while (i < qs.length()) {
            int eq = qs.indexOf('=', i);
            int amp = qs.indexOf('&', i);
            if (amp < 0) amp = qs.length();
            String key   = qs.substring(i, eq);
            String value = qs.substring(eq + 1, amp);

            if (key == "kp")       KP = value.toDouble();
            else if (key == "ki")  KI = value.toDouble();
            else if (key == "kd")  KD = value.toDouble();
            else if (key == "base") setBaseSpeed = value.toDouble();
            else if (key == "max_output") MAX_OUTPUT = value.toDouble();

            i = amp + 1;
        }

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println();
        client.print("{\"kp\":");   client.print(KP, 8);
        client.print(",\"ki\":");   client.print(KI, 8);
        client.print(",\"kd\":");   client.print(KD, 8);
        client.print(",\"base\":"); client.print(setBaseSpeed, 8);
        client.print(",\"max_output\":"); client.print(MAX_OUTPUT, 8);


        client.println("}");
        client.stop();
        return;
    }

    if (req.startsWith("GET /toggle")) {
        robotActive = !robotActive;

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println();
        client.print("{\"active\": ");
        client.print(robotActive ? "true" : "false");
        client.println("}");
        client.stop();
        return;
    }

    // Serve HTML + AJAX GUI
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE html><html><head><title>PID Tuner</title></head><body>");
    client.println("<h2>Portenta PID Tuner (AJAX)</h2>");

    client.print("KP: <input type='number' step='any' id='kp' value='"); client.print(KP, 8); client.println("'><br>");
    client.print("KI: <input type='number' step='any' id='ki' value='"); client.print(KI, 8); client.println("'><br>");
    client.print("KD: <input type='number' step='any' id='kd' value='"); client.print(KD, 8); client.println("'><br>");
    client.print("Base Speed: <input type='number' step='any' id='base' value='"); client.print(setBaseSpeed, 8); client.println("'><br>");
    client.print("Max Output: <input type='number' step='any' id='max_output' value='"); client.print(MAX_OUTPUT, 8); client.println("'><br>");



    client.println("<button onclick='updatePID()'>Update</button>");

    client.println("<button onclick='toggleRobot()'>Toggle Robot</button><br>");

    client.println("<p id='status'>Idle</p>");

    client.println(R"rawliteral(
    <script>
    function updatePID() {
        let kp   = document.getElementById("kp").value;
        let ki   = document.getElementById("ki").value;
        let kd   = document.getElementById("kd").value;
        let base = document.getElementById("base").value;
        let max_output = document.getElementById("max_output").value;

        let xhttp = new XMLHttpRequest();
        
        const url = `/update?kp=${kp}&ki=${ki}&kd=${kd}&base=${base}&max_output=${max_output}`;
        xhttp.open("GET", url, true);

        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                document.getElementById("status").innerText = "Updated!";
            }
        };
        xhttp.send();
    }
        
    function toggleRobot() {
        let xhttp = new XMLHttpRequest();
        xhttp.open("GET", "/toggle", true);
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                let response = JSON.parse(this.responseText);
                document.getElementById("status").innerText = "Robot is now " + (response.active ? "Active" : "Stopped");
            }
        };
        xhttp.send();
    }
    </script>

    </body></html>
    )rawliteral");

    client.flush();
    client.stop();
}