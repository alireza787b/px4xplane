(function (root) {
  "use strict";

  const DEFAULT_SCHEMA = {
    schema_version: 1,
    format: "px4xplane config.ini",
    runtime_supported_channel_types: ["float", "floatArray"],
    reload_policies: {
      live_reload: "Safe to change with Reload Config while PX4 is disconnected or connected. Do not change during an armed flight unless diagnosing.",
      reconnect_before_flight: "Change before connecting PX4 SITL, or disconnect/reconnect before flying. Reload may parse it, but in-flight changes are not supported.",
      px4_sitl_restart: "Requires changing PX4 airframe parameters or restarting PX4 SITL; not a px4xplane config.ini live setting.",
      development_only: "For controlled debugging only. Keep disabled for normal releases and demo flights."
    },
    global_fields: {
      config_name: {
        type: "string",
        required: true,
        group: "airframe",
        reload_policy: "reconnect_before_flight",
        description: "Name of the active airframe section in this config file."
      },
      debug_verbose_logging: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "live_reload",
        description: "Enable verbose plugin status logging."
      },
      debug_log_sensor_timing: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "development_only",
        description: "Log high-volume HIL_SENSOR timing details."
      },
      debug_log_sensor_values: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "development_only",
        description: "Log high-volume sensor values and noise diagnostics."
      },
      debug_log_ekf_innovations: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "development_only",
        description: "Log estimator-oriented diagnostics when available."
      },
      debug_log_accel_pipeline: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "development_only",
        description: "Log accelerometer pipeline stages."
      },
      debug_accel_bypass_calibration: {
        type: "bool",
        default: false,
        group: "sensor_contract",
        reload_policy: "development_only",
        description: "Bypass accelerometer calibration for controlled debugging."
      },
      diagnostic_log_enabled: {
        type: "bool",
        default: false,
        group: "diagnostics",
        reload_policy: "live_reload",
        description: "Emit compact bridge health lines to X-Plane Log.txt."
      },
      diagnostic_log_interval_s: {
        type: "float",
        default: 2.0,
        min: 0.25,
        max: 60.0,
        group: "diagnostics",
        reload_policy: "live_reload",
        description: "Seconds between compact bridge health log lines."
      },
      show_connection_status_hud: {
        type: "bool",
        default: true,
        group: "ui",
        reload_policy: "live_reload",
        description: "Show the compact connection/status HUD overlay."
      },
      fps_warning_enabled: {
        type: "bool",
        default: true,
        group: "ui",
        reload_policy: "live_reload",
        description: "Show a low-FPS warning when X-Plane frame rate drops below the configured threshold."
      },
      fps_warning_threshold: {
        type: "int",
        default: 50,
        min: 20,
        max: 120,
        group: "ui",
        reload_policy: "live_reload",
        description: "FPS threshold below which the low-FPS warning is shown."
      },
      accel_auto_calibrate: {
        type: "bool",
        default: false,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "Enable startup accelerometer gravity calibration."
      },
      accel_offset_x: {
        type: "float",
        default: 0.0,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "Manual accelerometer X-axis offset in m/s^2."
      },
      accel_offset_y: {
        type: "float",
        default: 0.0,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "Manual accelerometer Y-axis offset in m/s^2."
      },
      accel_offset_z: {
        type: "float",
        default: 0.0,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "Manual accelerometer Z-axis offset in m/s^2."
      },
      ground_stationary_accel_guard_enabled: {
        type: "bool",
        default: true,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "When stationary on the ground, replace X-Plane gear/contact g-load spikes with attitude-consistent gravity before publishing HIL_SENSOR acceleration."
      },
      ground_stationary_kinematics_guard_enabled: {
        type: "bool",
        default: true,
        group: "sensor_contract",
        reload_policy: "reconnect_before_flight",
        description: "When stationary on the ground, publish a coherent zero-motion contract: zero GPS/HIL_STATE velocity, zero gyro rates, and latched GPS/HIL_STATE position/altitude. Baro pressure remains live with tiny noise to avoid simulated stale-sensor detection."
      },
      mavlink_sensor_rate_hz: {
        type: "int",
        default: 200,
        min: 1,
        max: 500,
        group: "mavlink_rates",
        reload_policy: "reconnect_before_flight",
        description: "Target HIL_SENSOR rate. Actual rate is bounded by X-Plane callback rate."
      },
      mavlink_gps_rate_hz: {
        type: "int",
        default: 10,
        min: 1,
        max: 100,
        group: "mavlink_rates",
        reload_policy: "reconnect_before_flight",
        description: "Target HIL_GPS rate."
      },
      mavlink_state_rate_hz: {
        type: "int",
        default: 50,
        min: 1,
        max: 200,
        group: "mavlink_rates",
        reload_policy: "reconnect_before_flight",
        description: "Target HIL_STATE_QUATERNION rate."
      },
      mavlink_rc_rate_hz: {
        type: "int",
        default: 50,
        min: 1,
        max: 200,
        group: "mavlink_rates",
        reload_policy: "reconnect_before_flight",
        description: "Target HIL_RC_INPUTS rate."
      }
    },
    airframe_fields: {
      autoPropBrakes: {
        type: "motor_index_list",
        min: 0,
        max: 7,
        reload_policy: "reconnect_before_flight",
        description: "X-Plane engine indices that use the current configurable prop-brake behavior."
      },
      autoPropBrakeApplyThreshold: {
        type: "float",
        default: 0.01,
        min: 0,
        max: 1,
        reload_policy: "reconnect_before_flight",
        description: "Apply prop brakes only after every configured brake motor command stays at or below this normalized command."
      },
      autoPropBrakeReleaseThreshold: {
        type: "float",
        default: 0.12,
        min: 0,
        max: 1,
        reload_policy: "reconnect_before_flight",
        description: "Release all prop brakes immediately when any configured brake motor command reaches or exceeds this normalized command."
      },
      autoPropBrakeDwellSec: {
        type: "float",
        default: 2.0,
        min: 0,
        max: 30,
        reload_policy: "reconnect_before_flight",
        description: "Continuous low-command dwell time before configured prop brakes may apply."
      },
      autoPropBrakeMinAirspeedMps: {
        type: "float",
        default: 0,
        min: 0,
        max: 200,
        reload_policy: "reconnect_before_flight",
        description: "Optional true-airspeed gate for applying prop brakes. Zero disables the airspeed gate."
      },
      autoPropBrakeMode: {
        type: "string",
        default: "feather",
        enum: ["feather", "hard_lock", "prop_separate"],
        reload_policy: "reconnect_before_flight",
        description: "X-Plane prop-brake mechanism. feather writes normal prop actuator requests; hard_lock also writes low-level flightmodel prop geometry; prop_separate uses X-Plane prop-separation failures."
      },
      autoPropBrakeUseFailure: {
        type: "bool",
        default: false,
        reload_policy: "reconnect_before_flight",
        description: "Experimental. Also use X-Plane engine-seizure failure while prop brakes are active."
      },
      airspeedSource: {
        type: "string",
        default: "xplane_indicated",
        enum: ["xplane_indicated", "disabled", "body_axis"],
        reload_policy: "reconnect_before_flight",
        description: "Differential-pressure source for HIL_SENSOR. xplane_indicated converts X-Plane IAS with sea-level-equivalent density, disabled sends zero pressure, body_axis projects local air-relative velocity onto pitotAxisBody and uses local air density."
      },
      pitotAxisBody: {
        type: "string",
        default: "+X",
        enum: ["+X", "-X", "+Y", "-Y", "+Z", "-Z"],
        reload_policy: "reconnect_before_flight",
        description: "Physical pitot probe axis in PX4 body frame. Used only when airspeedSource is body_axis."
      },
      actuatorSmoothingTimeConstantSec: {
        type: "float",
        default: 0.0,
        min: 0.0,
        max: 1.0,
        reload_policy: "reconnect_before_flight",
        description: "Optional first-order smoothing time constant for normalized actuator commands before writing X-Plane datarefs. Use sparingly for low-rate HIL actuator streams and set 0 to disable."
      },
      actuatorSmoothingChannels: {
        type: "index_list",
        min: 0,
        max: 15,
        reload_policy: "reconnect_before_flight",
        description: "Optional comma-separated PX4 actuator channel list for actuator smoothing. Omit to smooth all channels when smoothing is enabled; set only surface channels to avoid delaying wheel steering, throttle, or flaps."
      },
      cameraViews: {
        type: "camera_views",
        default: "",
        reload_policy: "live_reload",
        description: "Optional camera presets for the active airframe. Format: Label|forward_m|right_m|up_m|pitch_offset_deg|heading_offset_deg|roll_offset_deg|zoom; repeat entries with semicolons."
      },
      channelN: {
        type: "actuator_mapping",
        min_channel: 0,
        max_channel: 15,
        reload_policy: "reconnect_before_flight",
        description: "PX4 actuator channel mapping to one or more X-Plane datarefs."
      }
    }
  };

  const RANGE_RE = /^\[\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*\]$/;
  const INDEX_RE = /^\[\s*(\d+(?:\s+\d+)*)\s*\]$/;
  const CAMERA_FIELDS = [
    ["label", "Label"],
    ["forward", "Forward m"],
    ["right", "Right m"],
    ["up", "Up m"],
    ["pitch", "Pitch deg"],
    ["heading", "Heading deg"],
    ["roll", "Roll deg"],
    ["zoom", "Zoom"]
  ];
  const DEFAULT_CONFIG_STATUS = "Load px4xplane/64/config.ini to edit the runtime configuration.";

  function createEmptyConfig() {
    return { globals: { config_name: "" }, sections: [] };
  }

  function clone(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function parseIni(text) {
    const config = createEmptyConfig();
    let current = null;

    for (const rawLine of text.split(/\r?\n/)) {
      const line = rawLine.trim();
      if (!line || line.startsWith(";") || line.startsWith("#")) continue;

      const sectionMatch = line.match(/^\[([^\]]+)\]$/);
      if (sectionMatch) {
        current = { name: sectionMatch[1].trim(), keys: {} };
        config.sections.push(current);
        continue;
      }

      const splitAt = line.indexOf("=");
      if (splitAt < 0) continue;
      const key = line.slice(0, splitAt).trim();
      const value = line.slice(splitAt + 1).trim();
      if (!key) continue;

      if (current) current.keys[key] = value;
      else config.globals[key] = value;
    }

    return config;
  }

  function sortedChannelKeys(keys) {
    return Object.keys(keys)
      .filter((key) => /^channel\d+$/.test(key))
      .sort((a, b) => Number(a.slice(7)) - Number(b.slice(7)));
  }

  function serializeIni(config) {
    const lines = [
      "; PX4-XPlane Configuration File",
      "; Generated by px4xplane config editor",
      ""
    ];

    for (const key of Object.keys(config.globals)) {
      lines.push(`${key} = ${config.globals[key]}`);
    }

    for (const section of config.sections) {
      lines.push("", `[${section.name}]`);

      if (Object.prototype.hasOwnProperty.call(section.keys, "autoPropBrakes")) {
        lines.push(`autoPropBrakes = ${section.keys.autoPropBrakes}`);
      }

      for (const key of Object.keys(section.keys)) {
        if (key === "autoPropBrakes" || /^channel\d+$/.test(key)) continue;
        if (key === "cameraViews" && !String(section.keys[key] || "").trim()) continue;
        lines.push(`${key} = ${section.keys[key]}`);
      }

      for (const key of sortedChannelKeys(section.keys)) {
        lines.push(`${key} = ${section.keys[key]}`);
      }
    }

    return `${lines.join("\n")}\n`;
  }

  function parseMappings(value) {
    return String(value || "")
      .split("|")
      .map((mapping) => mapping.trim())
      .filter(Boolean)
      .map((mapping) => {
        const parts = mapping.split(",").map((part) => part.trim());
        return {
          dataref: parts[0] || "",
          type: parts[1] || "float",
          indices: parts[2] || "0",
          range: parts[3] || "[0 1]"
        };
      });
  }

  function serializeMappings(mappings) {
    return mappings
      .map((mapping) => `${mapping.dataref}, ${mapping.type}, ${mapping.indices}, ${mapping.range}`)
      .join(" | ");
  }

  function parseIndices(value) {
    const trimmed = String(value || "").trim();
    if (!trimmed || trimmed === "0") return [];
    const match = INDEX_RE.exec(trimmed);
    if (!match) return null;
    return match[1].split(/\s+/).map((item) => Number.parseInt(item, 10));
  }

  function serializeIndices(indices) {
    if (!Array.isArray(indices) || indices.length === 0) return "0";
    return `[${indices.map((index) => Number.parseInt(index, 10)).join(" ")}]`;
  }

  function parseRangeValue(value) {
    const match = RANGE_RE.exec(String(value || "").trim());
    if (!match) return { min: "", max: "" };
    return { min: match[1], max: match[2] };
  }

  function serializeRange(minValue, maxValue) {
    return `[${String(minValue ?? "").trim()} ${String(maxValue ?? "").trim()}]`;
  }

  function looksLikeSchemaJson(text) {
    const trimmed = String(text || "").trim();
    if (!trimmed.startsWith("{")) return false;
    try {
      const parsed = JSON.parse(trimmed);
      return Boolean(parsed && (parsed.global_fields || parsed.airframe_fields || parsed.runtime_supported_channel_types));
    } catch (error) {
      return false;
    }
  }

  function isLikelyIniConfig(text) {
    const parsed = parseIni(text);
    return Boolean(parsed.globals.config_name || parsed.sections.length);
  }

  function parseCameras(value) {
    return String(value || "")
      .split(";")
      .map((entry) => entry.trim())
      .filter(Boolean)
      .map((entry) => {
        const parts = entry.split("|").map((part) => part.trim());
        return {
          label: parts[0] || "",
          forward: parts[1] || "0.0",
          right: parts[2] || "0.0",
          up: parts[3] || "0.0",
          pitch: parts[4] || "0.0",
          heading: parts[5] || "0.0",
          roll: parts[6] || "0.0",
          zoom: parts[7] || "1.0"
        };
      });
  }

  function serializeCameras(cameras) {
    return cameras
      .filter((camera) => Object.values(camera).some((value) => String(value || "").trim()))
      .map((camera) => [
        camera.label,
        camera.forward,
        camera.right,
        camera.up,
        camera.pitch,
        camera.heading,
        camera.roll,
        camera.zoom
      ].map((part) => String(part ?? "").trim()).join("|"))
      .join("; ");
  }

  function parseBool(value) {
    const normalized = String(value).trim().toLowerCase();
    if (["true", "1", "yes", "on"].includes(normalized)) return true;
    if (["false", "0", "no", "off"].includes(normalized)) return false;
    return null;
  }

  function addIssue(issues, level, location, message) {
    issues.push({ level, location, message });
  }

  function validateScalar(issues, key, value, field, locationPrefix = "global") {
    const location = `${locationPrefix}.${key}`;
    const trimmed = String(value || "").trim();
    if (field.required && !trimmed) {
      addIssue(issues, "error", location, "required value is empty");
      return;
    }

    if (field.type === "string") {
      if (Array.isArray(field.enum) && !field.enum.includes(trimmed)) {
        addIssue(issues, "error", location, `expected one of: ${field.enum.join(", ")}`);
      }
      return;
    }

    if (field.type === "camera_views") {
      if (!trimmed) return;
      const entries = trimmed.split(";").map((entry) => entry.trim()).filter(Boolean);
      if (entries.length > 8) {
        addIssue(issues, "error", location, "supports at most 8 camera views");
      }
      for (const [index, entry] of entries.entries()) {
        const parts = entry.split("|").map((part) => part.trim());
        if (parts.length !== 8) {
          addIssue(issues, "error", location, `camera ${index + 1} must have 8 pipe-separated fields`);
          continue;
        }
        if (!parts[0]) addIssue(issues, "error", location, `camera ${index + 1} label is empty`);
        for (const part of parts.slice(1)) {
          if (!Number.isFinite(Number.parseFloat(part))) {
            addIssue(issues, "error", location, `camera ${index + 1} has non-numeric field '${part}'`);
          }
        }
        const zoom = Number.parseFloat(parts[7]);
        if (Number.isFinite(zoom) && (zoom <= 0 || zoom > 4)) {
          addIssue(issues, "error", location, `camera ${index + 1} zoom must be > 0 and <= 4`);
        }
      }
      return;
    }

    if (field.type === "index_list") {
      const min = Number(field.min ?? 0);
      const max = Number(field.max ?? 15);
      if (!trimmed) return;
      for (const token of trimmed.split(",")) {
        const part = token.trim();
        if (!part) continue;
        const parsed = Number.parseInt(part, 10);
        if (!Number.isInteger(parsed) || String(parsed) !== part || parsed < min || parsed > max) {
          addIssue(issues, "error", location, `expected comma-separated indices ${min}..${max}`);
          return;
        }
      }
      return;
    }

    if (field.type === "bool") {
      if (parseBool(trimmed) === null) addIssue(issues, "error", location, "expected boolean");
      return;
    }

    if (field.type === "int" || field.type === "float") {
      const parsed = field.type === "int" ? Number.parseInt(trimmed, 10) : Number.parseFloat(trimmed);
      if (!Number.isFinite(parsed) || (field.type === "int" && String(parsed) !== trimmed)) {
        addIssue(issues, "error", location, `expected ${field.type}`);
        return;
      }
      if (field.min !== undefined && parsed < Number(field.min)) {
        addIssue(issues, "error", location, `below minimum ${field.min}`);
      }
      if (field.max !== undefined && parsed > Number(field.max)) {
        addIssue(issues, "error", location, `above maximum ${field.max}`);
      }
    }
  }

  function validateConfig(config, schema) {
    const activeSchema = schema || DEFAULT_SCHEMA;
    const issues = [];
    const fields = activeSchema.global_fields || {};
    const supportedTypes = new Set(activeSchema.runtime_supported_channel_types || ["float", "floatArray"]);

    for (const [key, field] of Object.entries(fields)) {
      if (field.required && !(key in config.globals)) {
        addIssue(issues, "error", `global.${key}`, "missing required field");
      }
    }

    for (const [key, value] of Object.entries(config.globals)) {
      if (!fields[key]) {
        addIssue(issues, "warning", `global.${key}`, "unknown global key");
        continue;
      }
      validateScalar(issues, key, value, fields[key]);
    }

    const active = String(config.globals.config_name || "").trim();
    if (!active) {
      addIssue(issues, "error", "global.config_name", "active airframe is empty");
    } else if (!config.sections.some((section) => section.name === active)) {
      addIssue(issues, "error", "global.config_name", `active airframe '${active}' has no section`);
    }

    for (const section of config.sections) {
      const channelKeys = sortedChannelKeys(section.keys);
      const airframeFields = activeSchema.airframe_fields || DEFAULT_SCHEMA.airframe_fields || {};
      if (section.name === active && channelKeys.length === 0) {
        addIssue(issues, "error", section.name, "active airframe has no channel mappings");
      }

      if (section.keys.autoPropBrakes) {
        for (const token of section.keys.autoPropBrakes.split(",")) {
          const trimmed = token.trim();
          if (!trimmed) continue;
          const index = Number.parseInt(trimmed, 10);
          if (!Number.isInteger(index) || String(index) !== trimmed || index < 0 || index > 7) {
            addIssue(issues, "error", `${section.name}.autoPropBrakes`, `invalid motor index '${trimmed}'`);
          }
        }
      }

      for (const [key, value] of Object.entries(section.keys)) {
        if (key === "autoPropBrakes" || /^channel\d+$/.test(key)) continue;
        const field = airframeFields[key];
        if (!field) {
          addIssue(issues, "warning", `${section.name}.${key}`, "unknown airframe key");
          continue;
        }
        validateScalar(issues, key, value, field, section.name);
      }

      if (section.keys.autoPropBrakeApplyThreshold && section.keys.autoPropBrakeReleaseThreshold) {
        const applyValue = Number.parseFloat(section.keys.autoPropBrakeApplyThreshold);
        const releaseValue = Number.parseFloat(section.keys.autoPropBrakeReleaseThreshold);
        if (Number.isFinite(applyValue) && Number.isFinite(releaseValue) && releaseValue <= applyValue) {
          addIssue(issues, "error", `${section.name}.autoPropBrakeReleaseThreshold`, "release threshold must be greater than apply threshold");
        }
      }

      for (const key of channelKeys) {
        const channel = Number.parseInt(key.slice(7), 10);
        if (channel < 0 || channel > 15) {
          addIssue(issues, "error", `${section.name}.${key}`, "channel outside 0..15");
          continue;
        }

        const mappings = parseMappings(section.keys[key]);
        if (mappings.length === 0) {
          addIssue(issues, "error", `${section.name}.${key}`, "channel has no mappings");
        }

        mappings.forEach((mapping, index) => {
          const location = `${section.name}.${key}[${index}]`;
          if (!mapping.dataref) addIssue(issues, "error", location, "empty dataref");
          if (!supportedTypes.has(mapping.type)) addIssue(issues, "error", location, `unsupported type '${mapping.type}'`);
          const indicesOk = mapping.indices === "0" || INDEX_RE.test(mapping.indices);
          if (!indicesOk) addIssue(issues, "error", location, "invalid array indices");
          if (mapping.type === "floatArray" && mapping.indices === "0") {
            addIssue(issues, "error", location, "floatArray requires at least one index");
          }
          const rangeMatch = RANGE_RE.exec(mapping.range);
          if (!rangeMatch) {
            addIssue(issues, "error", location, "invalid output range");
          } else if (Number(rangeMatch[1]) === Number(rangeMatch[2])) {
            addIssue(issues, "error", location, "output range endpoints are equal");
          }
        });
      }
    }

    return issues;
  }

  function findSection(config, name) {
    return config.sections.find((section) => section.name === name) || null;
  }

  function ensureSelectedSection(config, selectedName) {
    if (findSection(config, selectedName)) return selectedName;
    if (findSection(config, config.globals.config_name)) return config.globals.config_name;
    return config.sections.length ? config.sections[0].name : "";
  }

  function setChannelMapping(section, channel, mappings) {
    const key = `channel${channel}`;
    if (!mappings.length) delete section.keys[key];
    else section.keys[key] = serializeMappings(mappings);
  }

  function docsUrl(fieldName) {
    const base = "https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md";
    if (fieldName === "cameraViews") return `${base}#key-sections-explained`;
    if (/^channel/.test(fieldName) || fieldName === "channelN") return `${base}#channel-mappings`;
    return null;
  }

  function initEditor(documentRef) {
    const state = {
      schema: clone(DEFAULT_SCHEMA),
      config: createEmptyConfig(),
      selectedSection: "",
      configLoaded: false,
      configSource: "",
      autoStatus: DEFAULT_CONFIG_STATUS
    };

    const $ = (id) => documentRef.getElementById(id);

    function setStatus(message) {
      state.autoStatus = message;
      const source = $("sourceLine");
      if (source) source.textContent = message;
    }

    function bindValueEdit(element, handler) {
      element.addEventListener(element.tagName === "SELECT" ? "change" : "input", handler);
    }

    function schemaFields() {
      return state.schema.global_fields || {};
    }

    function refreshValidationAndPreview() {
      if (!state.configLoaded) {
        $("statusLine").textContent = "Config not loaded";
        $("statusLine").className = "status warn";
        $("issues").innerHTML = "<li class=\"ok\">No config loaded yet. Use Load config.ini to begin.</li>";
        $("preview").value = "";
        setStatus(state.autoStatus);
        return;
      }
      const issues = validateConfig(state.config, state.schema);
      const errorCount = issues.filter((issue) => issue.level === "error").length;
      const warningCount = issues.filter((issue) => issue.level === "warning").length;
      $("statusLine").textContent = errorCount
        ? `${errorCount} error(s), ${warningCount} warning(s)`
        : warningCount
          ? `${warningCount} warning(s)`
          : "Config OK";
      $("statusLine").className = errorCount ? "status bad" : warningCount ? "status warn" : "status ok";

      $("issues").innerHTML = issues.length
        ? issues.map((issue) => `<li class="${issue.level}"><b>${escapeHtml(issue.level)}</b> ${escapeHtml(issue.location)}: ${escapeHtml(issue.message)}</li>`).join("")
        : "<li class=\"ok\">No validation issues.</li>";
      $("preview").value = serializeIni(state.config);
      setStatus(state.autoStatus);
    }

    function renderAirframes() {
      state.selectedSection = ensureSelectedSection(state.config, state.selectedSection);
      const active = state.config.globals.config_name || "";
      if (!state.configLoaded) {
        $("airframes").innerHTML = "<p class=\"muted\">Load px4xplane/64/config.ini to show airframes. JSON is only optional schema metadata.</p>";
        return;
      }
      const orderedSections = [...state.config.sections].sort((a, b) => {
        if (a.name === active) return -1;
        if (b.name === active) return 1;
        return a.name.localeCompare(b.name);
      });
      const buttonForSection = (section) => {
        const selected = section.name === state.selectedSection ? "selected" : "";
        const activeClass = section.name === active ? " active" : "";
        return `<button class="airframe ${selected}${activeClass}" data-section="${escapeHtml(section.name)}">${escapeHtml(section.name)}</button>`;
      };
      $("airframes").innerHTML = `
        <div class="airframeGroupTitle">Airframes</div>
        ${orderedSections.map(buttonForSection).join("") || "<p class=\"muted\">No airframes configured.</p>"}
      `;

      for (const button of $("airframes").querySelectorAll("button")) {
        button.addEventListener("click", () => {
          state.selectedSection = button.dataset.section;
          render();
        });
      }
    }

    function helpCell(key, field) {
      const description = field.description || "";
      const reload = field.reload_policy || "";
      const url = docsUrl(key);
      return `<div class="helpCell">
        <button class="helpButton" title="${escapeHtml(description)}" aria-label="Help for ${escapeHtml(key)}">?</button>
        ${url ? `<a href="${url}" target="_blank" rel="noreferrer">Docs</a>` : ""}
        <span class="policy">${escapeHtml(reload)}</span>
      </div>`;
    }

    function inputForField(key, field, value, datasetName = "global") {
      const dataAttr = datasetName === "global" ? "data-global" : "data-airframe-field";
      if (field.type === "bool") {
        const selectedTrue = String(value).toLowerCase() === "true" ? "selected" : "";
        const selectedFalse = String(value).toLowerCase() === "false" ? "selected" : "";
        return `<select ${dataAttr}="${escapeHtml(key)}"><option value="true" ${selectedTrue}>true</option><option value="false" ${selectedFalse}>false</option></select>`;
      }
      if (Array.isArray(field.enum)) {
        const options = field.enum.map((item) => {
          const selected = String(item) === String(value) ? "selected" : "";
          return `<option value="${escapeHtml(item)}" ${selected}>${escapeHtml(item)}</option>`;
        });
        return `<select ${dataAttr}="${escapeHtml(key)}">${options.join("")}</select>`;
      }
      const type = field.type === "int" || field.type === "float" ? "number" : "text";
      const step = field.type === "int" ? "1" : "any";
      const bounds = `${field.min !== undefined ? ` min="${field.min}"` : ""}${field.max !== undefined ? ` max="${field.max}"` : ""}`;
      return `<input ${dataAttr}="${escapeHtml(key)}" type="${type}" step="${step}"${bounds} value="${escapeHtml(value)}">`;
    }

    function renderGlobals() {
      if (!state.configLoaded) {
        $("globalFields").innerHTML = `<div class="emptyState">
          <h3>Runtime config not loaded</h3>
          <p>Load <code>px4xplane/64/config.ini</code> with the main loader. The JSON file is only optional validation metadata and is loaded from the advanced schema control.</p>
        </div>`;
        return;
      }
      const fields = schemaFields();
      const rows = [];
      for (const [key, field] of Object.entries(fields)) {
        const value = state.config.globals[key] ?? field.default ?? "";
        rows.push(`<tr>
          <th>${escapeHtml(key)}</th>
          <td>${inputForField(key, field, value)}</td>
          <td>${helpCell(key, field)}</td>
        </tr>`);
      }
      $("globalFields").innerHTML = `<details class="editorGroup" open>
        <summary>Global Runtime Fields</summary>
        <div class="editorGroupBody">
          <table>
            <thead><tr><th>Setting</th><th>Value</th><th>Help</th></tr></thead>
            <tbody>${rows.join("")}</tbody>
          </table>
        </div>
      </details>`;
      for (const input of $("globalFields").querySelectorAll("[data-global]")) {
        bindValueEdit(input, () => {
          state.config.globals[input.dataset.global] = input.value;
          renderAirframes();
          refreshValidationAndPreview();
        });
      }
    }

    function renderCameraEditor(section) {
      const field = (state.schema.airframe_fields || DEFAULT_SCHEMA.airframe_fields).cameraViews || DEFAULT_SCHEMA.airframe_fields.cameraViews;
      const cameras = parseCameras(section.keys.cameraViews || "");
      const rows = cameras.map((camera, index) => `<tr data-camera-index="${index}">
        ${CAMERA_FIELDS.map(([key, label]) => `<td><label>${label}<input data-camera-field="${key}" value="${escapeHtml(camera[key])}"></label></td>`).join("")}
        <td><button data-remove-camera="1">Remove</button></td>
      </tr>`).join("");

      return `<div class="subsection">
        <div class="subsectionHeader">
          <div>
            <h3>Camera Views</h3>
            <p class="muted">Plugin-owned camera presets shown in X-Plane under PX4 X-Plane &gt; Camera Views. Up to 8 views are supported.</p>
          </div>
          ${helpCell("cameraViews", field)}
        </div>
        <table class="cameraTable">
          <thead><tr>${CAMERA_FIELDS.map(([, label]) => `<th>${escapeHtml(label)}</th>`).join("")}<th></th></tr></thead>
          <tbody>${rows || "<tr><td colspan=\"9\" class=\"muted\">No camera presets configured.</td></tr>"}</tbody>
        </table>
        <p><button id="addCamera">Add Camera</button></p>
      </div>`;
    }

    function readCameraRows() {
      const cameras = [];
      for (const row of $("airframeEditor").querySelectorAll("tr[data-camera-index]")) {
        const camera = {};
        for (const [key] of CAMERA_FIELDS) {
          const input = row.querySelector(`[data-camera-field="${key}"]`);
          camera[key] = input ? input.value : "";
        }
        cameras.push(camera);
      }
      return cameras;
    }

    function saveCameraRows(section) {
      const serialized = serializeCameras(readCameraRows());
      if (serialized) section.keys.cameraViews = serialized;
      else delete section.keys.cameraViews;
      refreshValidationAndPreview();
    }

    function renderSelectedAirframe() {
      const section = findSection(state.config, state.selectedSection);
      if (!state.configLoaded) {
        $("airframeEditor").innerHTML = `<div class="emptyState">
          <h3>Load config.ini</h3>
          <p>The plugin runtime file is <code>px4xplane/64/config.ini</code>. If your browser blocks automatic local-file loading, choose that INI file manually. JSON is only optional validation metadata.</p>
        </div>`;
        return;
      }
      if (!section) {
        $("airframeEditor").innerHTML = "<p class=\"muted\">No airframe selected. Load config.ini or add an airframe.</p>";
        return;
      }

      const airframeFields = state.schema.airframe_fields || DEFAULT_SCHEMA.airframe_fields || {};
      const isActive = section.name === state.config.globals.config_name;
      const scalarRows = [];
      for (const [key, field] of Object.entries(airframeFields)) {
        if (key === "autoPropBrakes" || key === "channelN" || key === "cameraViews") continue;
        const value = section.keys[key] ?? field.default ?? "";
        scalarRows.push(`<tr>
          <th>${escapeHtml(key)}</th>
          <td>${inputForField(key, field, value, "airframe")}</td>
          <td>${helpCell(key, field)}</td>
        </tr>`);
      }

      const channelRows = [];
      for (const key of sortedChannelKeys(section.keys)) {
        const channel = Number.parseInt(key.slice(7), 10);
        parseMappings(section.keys[key]).forEach((mapping, index) => {
          const range = parseRangeValue(mapping.range);
          const indices = parseIndices(mapping.indices);
          const indicesValue = Array.isArray(indices) ? indices.join(" ") : "";
          channelRows.push(`<tr data-channel="${channel}" data-index="${index}">
            <td>channel${channel}</td>
            <td><input data-map-field="dataref" value="${escapeHtml(mapping.dataref)}" placeholder="sim/flightmodel/..."></td>
            <td>${typeSelect(mapping.type)}</td>
            <td>
              ${indexInput(mapping.type, indicesValue)}
            </td>
            <td>
              <div class="rangeInput">
                <span class="bracketToken">[</span>
                <input data-map-field="rangeMin" type="number" step="any" value="${escapeHtml(range.min)}" placeholder="min">
                <input data-map-field="rangeMax" type="number" step="any" value="${escapeHtml(range.max)}" placeholder="max">
                <span class="bracketToken">]</span>
              </div>
            </td>
            <td><button data-remove-map="1">Remove</button></td>
          </tr>`);
        });
      }

      $("airframeEditor").innerHTML = `
        <div class="activeBanner">
          <div>
            <h3>Editing ${escapeHtml(section.name)}${isActive ? "<span class=\"activeBadge\">active</span>" : ""}</h3>
            <p class="muted">Active airframe is written to <code>config_name</code> and is used by px4xplane on reconnect.</p>
          </div>
          <div class="activeBannerActions">
            <button id="setActiveAirframe">${isActive ? "Active" : "Set Active"}</button>
            <button id="removeAirframe">Remove</button>
          </div>
        </div>
        <details class="editorGroup" open>
          <summary>Airframe Setup Fields</summary>
          <div class="editorGroupBody">
            <div class="metaGrid">
              <label>Section key <input id="airframeName" value="${escapeHtml(section.name)}"></label>
              <button id="renameAirframe">Rename</button>
            </div>
            <p class="muted">Setup-time fields should be changed before connecting PX4 SITL or before flight.</p>
            <label class="full">autoPropBrakes
              <input id="autoPropBrakes" value="${escapeHtml(section.keys.autoPropBrakes || "")}" placeholder="0, 1, 2, 3">
            </label>
            <table>
              <thead><tr><th>Setting</th><th>Value</th><th>Help</th></tr></thead>
              <tbody>${scalarRows.join("")}</tbody>
            </table>
          </div>
        </details>
        <details class="editorGroup" open>
          <summary>Actuator Mappings</summary>
          <div class="editorGroupBody">
            <p class="muted">Map PX4 output channels to X-Plane datarefs. Multiple mappings per channel are supported.</p>
            <div class="sectionBar compact">
              <label>Add channel <input id="newChannel" type="number" min="0" max="15" value="0"></label>
              <button id="addMapping">Add Mapping</button>
            </div>
            <table class="channelTable">
              <thead><tr><th>Channel</th><th>Dataref</th><th>Type</th><th>Indices</th><th>Output Range</th><th></th></tr></thead>
              <tbody>${channelRows.join("") || "<tr><td colspan=\"6\" class=\"muted\">No channel mappings configured.</td></tr>"}</tbody>
            </table>
          </div>
        </details>
        <details class="editorGroup">
          <summary>Camera Views</summary>
          <div class="editorGroupBody">${renderCameraEditor(section)}</div>
        </details>
      `;

      $("autoPropBrakes").addEventListener("input", (event) => {
        section.keys.autoPropBrakes = event.target.value;
        refreshValidationAndPreview();
      });
      for (const input of $("airframeEditor").querySelectorAll("[data-airframe-field]")) {
        bindValueEdit(input, () => {
          section.keys[input.dataset.airframeField] = input.value;
          refreshValidationAndPreview();
        });
      }
      $("renameAirframe").addEventListener("click", () => {
        const nextName = $("airframeName").value.trim();
        if (!nextName) return;
        if (state.config.globals.config_name === section.name) state.config.globals.config_name = nextName;
        section.name = nextName;
        state.selectedSection = nextName;
        render();
      });
      $("setActiveAirframe").addEventListener("click", () => {
        state.config.globals.config_name = section.name;
        render();
      });
      $("removeAirframe").addEventListener("click", () => {
        state.config.sections = state.config.sections.filter((item) => item !== section);
        if (state.config.globals.config_name === section.name) state.config.globals.config_name = "";
        state.selectedSection = "";
        render();
      });
      $("addCamera").addEventListener("click", () => {
        const cameras = parseCameras(section.keys.cameraViews || "");
        if (cameras.length >= 8) return;
        cameras.push({ label: "New View", forward: "0.0", right: "0.0", up: "0.0", pitch: "0.0", heading: "0.0", roll: "0.0", zoom: "0.90" });
        section.keys.cameraViews = serializeCameras(cameras);
        render();
      });
      for (const input of $("airframeEditor").querySelectorAll("[data-camera-field]")) {
        input.addEventListener("input", () => saveCameraRows(section));
      }
      for (const button of $("airframeEditor").querySelectorAll("[data-remove-camera]")) {
        button.addEventListener("click", () => {
          const row = button.closest("tr[data-camera-index]");
          if (row) row.remove();
          saveCameraRows(section);
          render();
        });
      }
      $("addMapping").addEventListener("click", () => {
        const channel = Math.max(0, Math.min(15, Number.parseInt($("newChannel").value, 10) || 0));
        const key = `channel${channel}`;
        const mappings = parseMappings(section.keys[key]);
        mappings.push({ dataref: "", type: "float", indices: "0", range: "[0 1]" });
        setChannelMapping(section, channel, mappings);
        render();
      });

      for (const row of $("airframeEditor").querySelectorAll("tr[data-channel]")) {
        const channel = Number.parseInt(row.dataset.channel, 10);
        const index = Number.parseInt(row.dataset.index, 10);
        for (const input of row.querySelectorAll("[data-map-field]")) {
          bindValueEdit(input, () => {
            const mappings = parseMappings(section.keys[`channel${channel}`]);
            const mapping = mappings[index];
            if (input.dataset.mapField === "indices") {
              const raw = input.value.trim();
              mapping.indices = serializeIndices(raw ? raw.split(/\s+/).map((item) => Number.parseInt(item, 10)).filter(Number.isFinite) : [0]);
            } else if (input.dataset.mapField === "rangeMin" || input.dataset.mapField === "rangeMax") {
              const minValue = row.querySelector('[data-map-field="rangeMin"]').value;
              const maxValue = row.querySelector('[data-map-field="rangeMax"]').value;
              mapping.range = serializeRange(minValue, maxValue);
            } else if (input.dataset.mapField === "type") {
              mapping.type = input.value;
              if (mapping.type === "float") {
                mapping.indices = "0";
              } else if (mapping.indices === "0") {
                mapping.indices = "[0]";
              }
              setChannelMapping(section, channel, mappings);
              render();
              return;
            } else {
              mapping[input.dataset.mapField] = input.value;
            }
            setChannelMapping(section, channel, mappings);
            refreshValidationAndPreview();
          });
        }
        row.querySelector("[data-remove-map]").addEventListener("click", () => {
          const mappings = parseMappings(section.keys[`channel${channel}`]);
          mappings.splice(index, 1);
          setChannelMapping(section, channel, mappings);
          render();
        });
      }
    }

    function typeSelect(value) {
      const types = state.schema.runtime_supported_channel_types || ["float", "floatArray"];
      return `<select data-map-field="type">${types.map((type) => {
        const selected = type === value ? "selected" : "";
        return `<option value="${type}" ${selected}>${type}</option>`;
      }).join("")}</select>`;
    }

    function indexInput(type, value) {
      if (type === "floatArray") {
        return `<div class="bracketInput">
          <span class="bracketToken">[</span>
          <input data-map-field="indices" value="${escapeHtml(value || "0")}" placeholder="0">
          <span class="bracketToken">]</span>
        </div>`;
      }

      return `<span class="singleToken">scalar</span>`;
    }

    function render() {
      renderAirframes();
      renderGlobals();
      renderSelectedAirframe();
      refreshValidationAndPreview();
    }

    async function loadText(url) {
      const response = await fetch(url, { cache: "no-store" });
      if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
      return response.text();
    }

    async function autoloadPackagedFiles() {
      const messages = [];
      try {
        const schemaText = await loadText("config_schema.json");
        state.schema = JSON.parse(schemaText);
        messages.push("schema loaded");
      } catch (error) {
        messages.push("using embedded schema");
      }
      try {
        const configText = await loadText("../64/config.ini");
        state.config = parseIni(configText);
        state.selectedSection = state.config.globals.config_name || "";
        state.configLoaded = true;
        state.configSource = "auto";
        messages.push("config.ini loaded");
      } catch (error) {
        messages.push("load config.ini manually if your browser blocks local file reads");
      }
      setStatus(`Auto-load: ${messages.join(", ")}.`);
      render();
    }

    $("configFile").addEventListener("change", (event) => {
      const file = event.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        const text = String(reader.result || "");
        if (looksLikeSchemaJson(text) || !isLikelyIniConfig(text)) {
          setStatus(`${file.name} is not a px4xplane runtime INI. Load px4xplane/64/config.ini here; schema JSON is optional metadata only.`);
          refreshValidationAndPreview();
          return;
        }
        state.config = parseIni(text);
        state.selectedSection = state.config.globals.config_name || "";
        state.configLoaded = true;
        state.configSource = file.name;
        setStatus(`Loaded runtime config from ${file.name}. Exported edits should replace px4xplane/64/config.ini.`);
        render();
      };
      reader.readAsText(file);
    });

    $("schemaFile").addEventListener("change", (event) => {
      const file = event.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const parsed = JSON.parse(String(reader.result || "{}"));
          if (!parsed.global_fields || !parsed.airframe_fields) {
            setStatus(`${file.name} is JSON, but not px4xplane schema metadata. Load config.ini in the main config loader.`);
            refreshValidationAndPreview();
            return;
          }
          state.schema = parsed;
          setStatus(`Loaded validation metadata from ${file.name}. Runtime config is still ${state.configSource || "not loaded"}.`);
          render();
        } catch (error) {
          setStatus(`${file.name} is not valid schema JSON. Runtime config is loaded from config.ini only.`);
          refreshValidationAndPreview();
        }
      };
      reader.readAsText(file);
    });

    $("autoloadConfig").addEventListener("click", () => {
      autoloadPackagedFiles();
    });

    $("addAirframe").addEventListener("click", () => {
      const nameInput = $("newAirframeName");
      const name = nameInput.value.trim();
      if (!name || findSection(state.config, name)) return;
      state.config.sections.push({ name, keys: {} });
      state.selectedSection = name;
      if (!state.config.globals.config_name) state.config.globals.config_name = name;
      nameInput.value = "";
      render();
    });

    $("exportConfig").addEventListener("click", () => {
      const text = serializeIni(state.config);
      $("preview").value = text;
      const blob = new Blob([text], { type: "text/plain" });
      const link = documentRef.createElement("a");
      link.href = URL.createObjectURL(blob);
      link.download = "config.ini";
      link.click();
      URL.revokeObjectURL(link.href);
    });

    $("copyPreview").addEventListener("click", async () => {
      $("preview").value = serializeIni(state.config);
      if (navigator.clipboard) await navigator.clipboard.writeText($("preview").value);
    });

    $("themeToggle").addEventListener("click", () => {
      const current = documentRef.documentElement.dataset.theme || "system";
      const next = current === "dark" ? "light" : "dark";
      documentRef.documentElement.dataset.theme = next;
      localStorage.setItem("px4xplaneEditorTheme", next);
    });

    const storedTheme = localStorage.getItem("px4xplaneEditorTheme");
    if (storedTheme) documentRef.documentElement.dataset.theme = storedTheme;
    else documentRef.documentElement.dataset.theme = "system";

    render();
    autoloadPackagedFiles();
  }

  function escapeHtml(value) {
    return String(value ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  const api = {
    DEFAULT_SCHEMA,
    createEmptyConfig,
    parseIni,
    serializeIni,
    parseMappings,
    serializeMappings,
    parseIndices,
    serializeIndices,
    parseRangeValue,
    serializeRange,
    parseCameras,
    serializeCameras,
    validateConfig
  };

  if (typeof module !== "undefined" && module.exports) module.exports = api;
  root.PX4XPlaneConfigEditor = api;

  if (typeof document !== "undefined") {
    document.addEventListener("DOMContentLoaded", () => initEditor(document));
  }
})(typeof window !== "undefined" ? window : globalThis);
