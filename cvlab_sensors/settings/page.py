"""Settings page: edit the sensor settings (cvlab_sensors.settings.store) with a live preview.

Saved settings apply the next time a sensor opens. The page opens the sensors of the tab shown (like
a capture page) and shows, for each of them, the values it runs with next to the saved ones, so a
saved change that is not active yet is visible. The Hesai settings are only read: its tab compares
what it reports with the expected values (its preview streams its points like the others). While a
recording runs the page is read-only.
"""

import io

import cv2
from flask import Blueprint, jsonify, render_template, request, send_file

from cvlab_sensors import config
from cvlab_sensors.replay import views
from cvlab_sensors.sensors import hesai
from cvlab_sensors.settings import store
from cvlab_sensors.web import jpeg, preview

LIVOX_PREVIEW_S = 1.0   # the non-repetitive pattern fills in over time
HESAI_PREVIEW_S = 0.1   # one revolution at 10 Hz


def running(sensors, recording):
    """{output: {"label", "state" (closed / opening / open), "where", "report", "pending", "error"}}:
    what every sensor runs with now, from the recorders when they record, else from the live sensors."""
    outputs = {}
    for group, names in store.GROUP_OUTPUTS.items():
        for idx, output in enumerate(names):
            entry = {"label": store.OUTPUT_LABELS[output], "group": group, "state": "closed", "where": None,
                     "report": None, "error": None}
            recorder = recording.outputs.get(output)
            if group == "hesai":
                # Read over the network whether it records or not: nothing to open
                result = recording.hesai.latest() if recording.hesai else {"readback": None, "error": "not read"}
                if result["readback"] is not None:
                    entry.update(state="open", where="read over the network",
                                 report=store.applied_report("hesai", store.load("hesai"), result["readback"],
                                                             hesai.problems(result["readback"])))
                entry["error"] = result["error"]
            elif recorder is not None and recorder.is_running() and recording.session is not None:
                report = store.read_applied(recording.session, output)
                entry.update(where="Record page", report=report, state="open" if report else "opening")
            elif group == "rgb" and sensors.rgb.running():
                report = sensors.rgb.applied(idx)
                # While opening, the settings snapshot it opens with is already known
                report = report or {"values": sensors.rgb.settings, "readback": {}, "notes": []}
                entry.update(where=where(sensors), report=report,
                             state="open" if sensors.rgb.applied(idx) else "opening", error=sensors.rgb.error)
            elif group == "event" and sensors.event[output].running():
                cam = sensors.event[output]
                try:
                    report = cam.applied()
                except RuntimeError as e:
                    report, entry["error"] = None, str(e)
                entry.update(where=where(sensors), state="open" if report else "opening",
                             report=report or {"values": cam.settings, "readback": {}, "notes": []})
            elif group == "livox" and sensors.livox.running():
                report = sensors.livox.applied()
                entry.update(where=where(sensors), report=report, error=sensors.livox.error,
                             state="open" if report and report["readback"] else "opening")
            entry["pending"] = store.pending(group, entry["report"])
            outputs[output] = entry
    return outputs


def where(sensors):
    return "Settings preview" if (sensors.tool or "").startswith("settings") else "Calibration capture"


def summary(recording):
    """For the Record and home pages: per group the saved values that differ from the defaults, the
    recording sensors still running with older settings, and what they could not set."""
    result = {}
    for group, label in store.GROUPS.items():
        values = store.load(group)
        stale, notes = [], []
        for output in store.GROUP_OUTPUTS[group]:
            recorder = recording.outputs.get(output)
            if group == "hesai" or recording.session is None or recorder is None or not recorder.is_running():
                continue   # the Hesai differences are in warnings()
            report = store.read_applied(recording.session, output)
            if store.pending(group, report):
                stale.append(store.OUTPUT_LABELS[output])
            notes += [f"{store.OUTPUT_LABELS[output]}: {note}" for note in (report or {}).get("notes", [])]
        result[group] = {
            "label": label,
            "kind": store.GROUP_KIND[group],
            "changed": [f"{f['label']} {store.format_value(f, values[f['key']])}" for f in store.FIELDS[group]
                        if values[f["key"]] != f["default"] and store.applies(f, values)],
            "saved_at": store.saved_at(group),
            "stale": stale,
            "notes": notes,
        }
    return result


def hesai_warnings(result):
    """The Hesai settings that differ from the expected ones, and the ones that break recordings."""
    readback = (result or {}).get("readback")
    if not readback:
        return []
    found = [f"Hesai {label} is {reported}, expected {expected}: change it on its web page "
             f"({store.HESAI_WEB_PAGE}), or the expected value on the Settings page"
             for label, reported, expected in store.hesai_mismatches(readback)]
    return found + [f"Hesai {problem}" for problem in hesai.problems(readback)]


def warnings(recording):
    """Warnings for every page: the Hesai differs from what we expect, a calibration was made with
    other settings."""
    return hesai_warnings(recording.hesai.latest() if recording.hesai else None) + store.calibration_mismatches()


def blueprint(sensors, recording):
    bp = Blueprint("settings", __name__, url_prefix="/settings")

    def locked():
        return sensors.blocked()

    @bp.route("")
    def page():
        # The sensors of the tab shown are opened by the page (api/use)
        return render_template("settings.html", groups=list(store.GROUPS.items()), fields=store.FIELDS,
                               kinds=store.GROUP_KIND, outputs=store.GROUP_OUTPUTS, output_labels=store.OUTPUT_LABELS,
                               hesai_web_page=store.HESAI_WEB_PAGE,
                               settings_file=str(config.SETTINGS_FILE.relative_to(config.REPO_ROOT))
                               if config.SETTINGS_FILE.is_relative_to(config.REPO_ROOT) else str(config.SETTINGS_FILE))

    @bp.route("/api/use", methods=["POST"])
    def use():
        """Open the sensors of a tab (and close the others)."""
        group = request.get_json(force=True).get("group")
        if group not in store.GROUPS:
            return jsonify({"error": f"unknown settings group {group!r}"}), 400
        if not locked():
            sensors.use(f"settings_{group}")
        return jsonify({"status": "ok"})

    @bp.route("/api/hesai/refresh", methods=["POST"])
    def hesai_refresh():
        return jsonify(recording.hesai.refresh() if recording.hesai else {"error": "no Hesai monitor"})

    @bp.route("/api/status")
    def status():
        rates = {}
        if sensors.rgb.running():
            for idx, output in enumerate(store.GROUP_OUTPUTS["rgb"]):
                rates[output] = sensors.rgb.status(idx)
        for output, cam in sensors.event.items():
            if cam.running():
                try:
                    rates[output] = cam.status()
                except RuntimeError as e:
                    rates[output] = {"rate": 0, "unit": "Mev/s", "error": str(e), "warning": None}
        for output, stream in [("lidar_livox", sensors.livox), ("lidar_hesai", sensors.hesai)]:
            if stream.running():
                rates[output] = stream.status()
        hesai_result = recording.hesai.latest() if recording.hesai else None
        return jsonify({
            "saved": {group: {"values": store.load(group), "saved_at": store.saved_at(group),
                              "warnings": store.validate(group, store.load(group))[2]} for group in store.GROUPS},
            "outputs": running(sensors, recording),
            "rates": rates,
            "locked": locked(),
            "preview": (sensors.tool or "").startswith("settings"),
            "hesai": {**(hesai_result or {}), "warnings": hesai_warnings(hesai_result)},
            "calibration": store.calibration_mismatches(),
        })

    def body():
        data = request.get_json(force=True)
        group = data.get("group")
        if group not in store.GROUPS:
            raise ValueError(f"unknown settings group {group!r}")
        return group, data.get("values", {})

    @bp.route("/api/validate", methods=["POST"])
    def validate():
        try:
            group, values = body()
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        _, errors, warnings = store.validate(group, values)
        return jsonify({"errors": errors, "warnings": warnings})

    @bp.route("/api/save", methods=["POST"])
    def save():
        if locked():
            return jsonify({"error": f"settings are read-only: {locked()}"}), 409
        try:
            group, values = body()
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        values, errors, warnings = store.save(group, values)
        if errors:
            return jsonify({"error": "not saved: fix the fields in red", "errors": errors, "warnings": warnings}), 400
        return jsonify({"values": values, "warnings": warnings, "saved_at": store.saved_at(group)})

    @bp.route("/api/reset", methods=["POST"])
    def reset():
        if locked():
            return jsonify({"error": f"settings are read-only: {locked()}"}), 409
        try:
            group, _ = body()
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        store.reset(group)
        return jsonify({"values": store.load(group)})

    @bp.route("/api/restart", methods=["POST"])
    def restart():
        """Reopen the preview sensors whose settings are older than the saved ones."""
        if locked():
            return jsonify({"error": locked()}), 409
        if sensors.tool:
            sensors.use(sensors.tool)
        return jsonify({"status": "ok"})

    @bp.route("/api/preview/<output>")
    def live_preview(output):
        if not (sensors.tool or "").startswith("settings"):
            return jsonify({"error": "preview not open"}), 503
        try:
            if output in store.GROUP_OUTPUTS["rgb"]:
                frame = sensors.rgb.latest(store.GROUP_OUTPUTS["rgb"].index(output))
                image = None if frame is None else cv2.cvtColor(frame[1], cv2.COLOR_RGB2BGR)
            elif output in sensors.event:
                image = sensors.event[output].preview_frame()
            elif output == "lidar_livox":
                points = sensors.livox.latest_points(LIVOX_PREVIEW_S)
                image = None if points is None or not len(points[0]) else views.livox_view(*points)
            elif output == "lidar_hesai":
                points = sensors.hesai.latest_points(HESAI_PREVIEW_S)
                image = None if points is None or not len(points[0]) else views.panorama(*points)
            else:
                return jsonify({"error": f"unknown camera {output}"}), 404
        except RuntimeError as e:
            return jsonify({"error": str(e)}), 503
        if image is None:
            return jsonify({"error": "no image yet"}), 503
        response = send_file(io.BytesIO(jpeg(preview(image))), mimetype="image/jpeg")
        response.headers["Cache-Control"] = "no-store"
        return response

    return bp
