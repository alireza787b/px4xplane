const assert = require("assert");
const editor = require("../docs/assets/config-editor.js");

const schema = {
  runtime_supported_channel_types: ["float", "floatArray"],
  global_fields: {
    config_name: { type: "string", required: true, reload_policy: "reconnect_before_flight" },
    diagnostic_log_enabled: { type: "bool", default: true, reload_policy: "live_reload" },
    fps_warning_threshold: { type: "int", min: 20, max: 120, reload_policy: "live_reload" }
  }
};

const source = `
config_name = Alia250
diagnostic_log_enabled = true
fps_warning_threshold = 50

[Alia250]
autoPropBrakes =
autoPropBrakeApplyThreshold = 0.01
autoPropBrakeReleaseThreshold = 0.12
autoPropBrakeDwellSec = 2.0
autoPropBrakeMinAirspeedMps = 55.0
autoPropBrakeMode = feather
autoPropBrakeUseFailure = false
channel0 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]
channel4 = sim/flightmodel/controls/wing2l_ail1def, float, 0, [-20 10] | sim/flightmodel/controls/wing2l_ail2def, float, 0, [-20 10]
`;

const config = editor.parseIni(source);
assert.strictEqual(config.globals.config_name, "Alia250");
assert.strictEqual(config.sections.length, 1);
assert.strictEqual(editor.parseMappings(config.sections[0].keys.channel4).length, 2);

const serialized = editor.serializeIni(config);
assert(serialized.includes("[Alia250]"));
assert(serialized.includes("channel0 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]"));

const validIssues = editor.validateConfig(config, schema);
assert.deepStrictEqual(validIssues.filter((issue) => issue.level === "error"), []);

const bad = editor.parseIni(`
config_name = Missing
diagnostic_log_enabled = maybe
fps_warning_threshold = 5
unknown_key = 1

[Aircraft]
channel0 = , floatArray, 0, [1 1]
autoPropBrakes = 9
autoPropBrakeApplyThreshold = 0.2
autoPropBrakeReleaseThreshold = 0.1
autoPropBrakeMode = broken
`);

const messages = editor.validateConfig(bad, schema).map((issue) => `${issue.level}:${issue.location}:${issue.message}`);
assert(messages.some((message) => message.includes("active airframe 'Missing'")));
assert(messages.some((message) => message.includes("expected boolean")));
assert(messages.some((message) => message.includes("below minimum 20")));
assert(messages.some((message) => message.includes("unknown global key")));
assert(messages.some((message) => message.includes("invalid motor index '9'")));
assert(messages.some((message) => message.includes("release threshold must be greater")));
assert(messages.some((message) => message.includes("expected one of: feather, hard_lock, prop_separate")));
assert(messages.some((message) => message.includes("empty dataref")));
assert(messages.some((message) => message.includes("floatArray requires")));
assert(messages.some((message) => message.includes("output range endpoints are equal")));

console.log("config editor tests passed");
