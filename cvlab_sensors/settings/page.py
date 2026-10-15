"""Settings page: edit the sensor settings (cvlab_sensors.settings.store) with a live preview.

Saved settings apply the next time a sensor opens. The page opens the cameras itself (like a
capture page) and shows, for every camera, the values it runs with next to the saved ones, so
a saved change that is not active yet is visible. While a recording runs the page is read-only.
"""

import io

import cv2
from flask import Blueprint, jsonify, render_template, request, send_file

from cvlab_sensors import config
from cvlab_sensors.settings import store
from cvlab_sensors.web import jpeg, preview


def format_value(f, value):
    if value is None:
        return "no cap" if "rate" in f["key"] else "camera default"
    if f["kind"] == "bool":
        return "on" if value else "off"
    if f["kind"] == "choice":
        return dict(f["choices"]).get(value, value)
    return f"{value:g} {f['unit']}".strip()


def running(sensors, recording):
    """{output: {"label", "state" (closed / opening / open / error), "where", "report", "pending"}}:
    what every camera runs with now, from the recorders when they record, else from the live sensors."""
    outputs = {}
    for group, names in store.GROUP_OUTPUTS.items():
        for idx, output in enumerate(names):
            entry = {"label": store.OUTPUT_LABELS[output], "group": group, "state": "closed", "where": None,
                     "report": None, "error": None}
            recorder = recording.outputs.get(output)
            if recorder is not None and recorder.is_running() and recording.session is not None:
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
            entry["pending"] = store.pending(group, entry["report"])
            outputs[output] = entry
    return outputs


def where(sensors):
    return "Settings preview" if sensors.tool == "settings" else "Calibration capture"


def summary(recording):
    """For the Record and home pages: per group the saved values that differ from the defaults, the
    recording cameras still running with older settings, and what the cameras could not set."""
    result = {}
    for group, label in store.GROUPS.items():
        values = store.load(group)
        stale, notes = [], []
        for output in store.GROUP_OUTPUTS[group]:
            recorder = recording.outputs.get(output)
            if recording.session is None or recorder is None or not recorder.is_running():
                continue
            report = store.read_applied(recording.session, output)
            if store.pending(group, report):
                stale.append(store.OUTPUT_LABELS[output])
            notes += [f"{store.OUTPUT_LABELS[output]}: {note}" for note in (report or {}).get("notes", [])]
        result[group] = {
            "label": label,
            "changed": [f"{f['label']} {format_value(f, values[f['key']])}" for f in store.FIELDS[group]
                        if values[f["key"]] != f["default"] and store.applies(f, values)],
            "saved_at": store.saved_at(group),
            "stale": stale,
            "notes": notes,
        }
    return result


def blueprint(sensors, recording):
    bp = Blueprint("settings", __name__, url_prefix="/settings")

    def locked():
        return sensors.blocked()

    @bp.route("")
    def page():
        if not locked():
            sensors.use("settings")
        return render_template("settings.html", groups=list(store.GROUPS.items()), fields=store.FIELDS,
                               outputs=store.GROUP_OUTPUTS, output_labels=store.OUTPUT_LABELS,
                               settings_file=str(config.SETTINGS_FILE.relative_to(config.REPO_ROOT))
                               if config.SETTINGS_FILE.is_relative_to(config.REPO_ROOT) else str(config.SETTINGS_FILE))

    @bp.route("/api/status")
    def status():
        rates = {}
        if sensors.tool == "settings":
            for idx, output in enumerate(store.GROUP_OUTPUTS["rgb"]):
                rates[output] = sensors.rgb.status(idx)
            for output, cam in sensors.event.items():
                try:
                    rates[output] = cam.status()
                except RuntimeError as e:
                    rates[output] = {"rate": 0, "unit": "Mev/s", "error": str(e), "warning": None}
        return jsonify({
            "saved": {group: {"values": store.load(group), "saved_at": store.saved_at(group),
                              "warnings": store.validate(group, store.load(group))[2]} for group in store.GROUPS},
            "outputs": running(sensors, recording),
            "rates": rates,
            "locked": locked(),
            "preview": sensors.tool == "settings",
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
        """Reopen the preview cameras whose settings are older than the saved ones."""
        if locked():
            return jsonify({"error": locked()}), 409
        sensors.use("settings")
        return jsonify({"status": "ok"})

    @bp.route("/api/preview/<output>")
    def live_preview(output):
        if sensors.tool != "settings":
            return jsonify({"error": "preview not open"}), 503
        try:
            if output in store.GROUP_OUTPUTS["rgb"]:
                frame = sensors.rgb.latest(store.GROUP_OUTPUTS["rgb"].index(output))
                image = None if frame is None else cv2.cvtColor(frame[1], cv2.COLOR_RGB2BGR)
            elif output in sensors.event:
                image = sensors.event[output].preview_frame()
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
