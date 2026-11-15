#!/usr/bin/env python3
"""Generate C and Python protocol bindings from shared/robot_spec.yaml + protocol.yaml.

Outputs (both are checked-in build artefacts, never edit by hand):
    firmware_esp32p4/components/generated/include/robot_protocol.h
    brain_pi/src/generated/robot_protocol.py

Usage:  python shared/generate_bindings.py [--check]
    --check   regenerate in memory and fail (exit 1) if the files on disk differ.

The joint count N is derived from the length of `joints:` in robot_spec.yaml; no
other file hard-codes it.
"""
from __future__ import annotations

import argparse
import hashlib
import keyword
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required: pip install pyyaml  (or `uv sync` in brain_pi/)")

SHARED = Path(__file__).resolve().parent
ROOT = SHARED.parent
SPEC_PATH = SHARED / "robot_spec.yaml"
PROTO_PATH = SHARED / "protocol.yaml"
CALIB_PATH = SHARED / "calibration.yaml"  # optional, written by tools/servo_zero_calibrator.py
OUT_C = ROOT / "firmware_esp32p4" / "components" / "generated" / "include" / "robot_protocol.h"
OUT_PY = ROOT / "brain_pi" / "src" / "generated" / "robot_protocol.py"

# yaml type -> (C type, struct format char, size in bytes)
SCALARS = {
    "u8": ("uint8_t", "B", 1), "u16": ("uint16_t", "H", 2), "u32": ("uint32_t", "I", 4),
    "u64": ("uint64_t", "Q", 8), "i8": ("int8_t", "b", 1), "i16": ("int16_t", "h", 2),
    "i32": ("int32_t", "i", 4), "f32": ("float", "f", 4), "f64": ("double", "d", 8),
}
ROLES = {"neck", "hip", "knee", "ankle", "other"}
GAZE_AXES = {"yaw": 1, "pitch": 2}
IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class SpecError(Exception):
    pass


def snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def need(cond: bool, msg: str) -> None:
    if not cond:
        raise SpecError(msg)


# --------------------------------------------------------------------------- loading
def spec_hash() -> str:
    """Fingerprint of all inputs; embedded in both outputs so staleness is detectable."""
    h = hashlib.sha256()
    for p in (SPEC_PATH, PROTO_PATH, CALIB_PATH):
        if p.exists():
            h.update(p.read_bytes())
    return h.hexdigest()[:16]


def load_robot() -> dict:
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    joints = spec.get("joints") or []
    need(1 <= len(joints) <= 32, "joints: need between 1 and 32 entries")
    names, ids = set(), set()
    for j in joints:
        n = j.get("name")
        need(isinstance(n, str) and IDENT.match(n) and not keyword.iskeyword(n), f"bad joint name {n!r}")
        need(n not in names, f"duplicate joint name {n}")
        names.add(n)
        need(isinstance(j.get("id"), int) and 0 <= j["id"] <= 252, f"{n}: id must be int 0..252")
        need(j["id"] not in ids, f"{n}: duplicate servo id {j['id']}")
        ids.add(j["id"])
        need(j.get("role") in ROLES, f"{n}: role must be one of {sorted(ROLES)}")
        for k in ("min_rad", "max_rad", "neutral_rad", "max_current_ma"):
            need(isinstance(j.get(k), (int, float)), f"{n}: missing numeric {k}")
        need(j["min_rad"] < j["neutral_rad"] < j["max_rad"] or j["min_rad"] <= j["neutral_rad"] <= j["max_rad"],
             f"{n}: neutral_rad must lie within [min_rad, max_rad]")
        need(j["min_rad"] < j["max_rad"], f"{n}: min_rad must be < max_rad")
        need(j["max_current_ma"] > 0, f"{n}: max_current_ma must be > 0")
        j.setdefault("max_vel_rad_s", 5.0)
        need(j["max_vel_rad_s"] > 0, f"{n}: max_vel_rad_s must be > 0")
        if j["role"] == "neck":
            need(j.get("gaze_axis") in GAZE_AXES, f"{n}: neck joints need gaze_axis yaw|pitch")
    need(spec["bus"]["driver"] in ("feetech_sts", "dynamixel_x"), "bus.driver must be feetech_sts|dynamixel_x")
    for rname, r in (spec.get("reflexes") or {}).items():
        for jn in (r.get("offsets_rad") or {}):
            need(jn in names, f"reflexes.{rname}.offsets_rad references unknown joint {jn}")
    # Optional calibration (zero offsets measured by servo_zero_calibrator.py).
    offsets = {n: 0.0 for n in names}
    if CALIB_PATH.exists():
        cal = yaml.safe_load(CALIB_PATH.read_text(encoding="utf-8")) or {}
        for jn, v in (cal.get("zero_offset_rad") or {}).items():
            need(jn in names, f"calibration.yaml references unknown joint {jn}")
            offsets[jn] = float(v)
    for j in joints:
        j["zero_offset_rad"] = offsets[j["name"]]
    return spec


def load_protocol(num_joints: int) -> dict:
    proto = yaml.safe_load(PROTO_PATH.read_text(encoding="utf-8"))
    consts = {"NUM_JOINTS": num_joints}
    consts.update({k: int(v) for k, v in (proto.get("constants") or {}).items()})
    proto["_consts"] = consts
    structs = proto.get("structs") or {}
    type_ids = set()

    def resolve(expr) -> int:
        """count may be an int, a constant name, or a '*' product of those."""
        total = 1
        for tok in str(expr).split("*"):
            tok = tok.strip()
            if tok.isdigit():
                total *= int(tok)
            else:
                need(tok in consts, f"unknown constant {tok!r} in count {expr!r}")
                total *= consts[tok]
        return total

    def layout(fields, ctx):
        out = []
        for f in fields:
            t, cnt = f["type"], f.get("count")
            need(IDENT.match(f["name"]) is not None, f"{ctx}: bad field name {f['name']}")
            need(t in SCALARS or t in structs, f"{ctx}.{f['name']}: unknown type {t}")
            n = resolve(cnt) if cnt is not None else None
            need(n is None or n >= 1, f"{ctx}.{f['name']}: count resolved to {n}")
            if t in SCALARS:
                esize = SCALARS[t][2]
            else:
                esize = sum(SCALARS[x["type"]][2] for x in structs[t])
            out.append({**f, "n": n, "expr": cnt, "esize": esize, "is_struct": t in structs})
        return out

    proto["_structs"] = {n: layout(f, n) for n, f in structs.items()}
    pk = {}
    for pname, p in proto["packets"].items():
        need(p["msg_type"] not in type_ids, f"duplicate msg_type for {pname}")
        type_ids.add(p["msg_type"])
        lay = layout(p["fields"], pname)
        size = sum((f["n"] or 1) * f["esize"] for f in lay)
        need(size <= 65535, f"{pname} payload too large")
        pk[pname] = {"msg_type": p["msg_type"], "fields": lay, "size": size}
    proto["_packets"] = pk
    return proto


# --------------------------------------------------------------------------- helpers
def cfloat(v) -> str:
    s = repr(float(v))
    return s + "f"


def c_count(expr) -> str:
    """Translate a count expression to C: every identifier gets the ROBOT_ prefix."""
    return re.sub(r"[A-Za-z_][A-Za-z0-9_]*", lambda m: "ROBOT_" + m.group(0), str(expr))


def jarr_c(ctype: str, name: str, values, fmt) -> str:
    body = ", ".join(fmt(v) for v in values)
    return f"static const {ctype} {name}[ROBOT_NUM_JOINTS] = {{ {body} }};"


# --------------------------------------------------------------------------- C emitter
def emit_c(spec: dict, proto: dict) -> str:
    J = spec["joints"]
    N = len(J)
    structs, packets = proto["_structs"], proto["_packets"]
    o = []
    w = o.append
    w("/* AUTO-GENERATED by shared/generate_bindings.py -- DO NOT EDIT.")
    w(" * Sources: shared/robot_spec.yaml, shared/protocol.yaml, shared/calibration.yaml (optional)")
    w(f" * Robot: {spec.get('robot_name', '?')}   joints: {N}   spec hash: {spec_hash()}")
    w(" */")
    w("#pragma once")
    w("#include <stdint.h>\n#include <stddef.h>\n#include <stdbool.h>\n#include <string.h>\n#include <assert.h>\n")
    w("#ifdef __cplusplus\nextern \"C\" {\n#endif\n")
    w('static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "wire format is little-endian; host must be too");\n')
    w(f'#define ROBOT_SPEC_HASH "{spec_hash()}"')
    w(f"#define ROBOT_NUM_JOINTS {N}")
    for k, v in proto["constants"].items():
        w(f"#define ROBOT_{k} {int(v)}")
    ctl = spec["control"]
    w(f"#define ROBOT_STATE_HZ {int(ctl['state_hz'])}")
    w(f"#define ROBOT_ACTION_HZ {int(ctl['action_hz'])}")
    w(f"#define ROBOT_ACTION_TIMEOUT_MS {int(ctl['action_timeout_ms'])}")
    w(f"#define ROBOT_BUS_BAUD {int(spec['bus']['baud'])}")
    w(f"#define ROBOT_BUS_DRIVER_{spec['bus']['driver'].upper()} 1")
    w(f"#define ROBOT_MAGIC {int(proto['framing']['magic'])}u")
    w(f"#define ROBOT_CRC_INIT {int(proto['framing']['crc']['init'])}u")
    w(f"#define ROBOT_CRC_POLY {int(proto['framing']['crc']['poly'])}u\n")

    # enums
    for ename, e in proto["enums"].items():
        sn = snake(ename).upper()
        if e.get("flags"):
            for k, bit in e["values"].items():
                w(f"#define ROBOT_{sn}_{k} (1u << {int(bit)})")
            w("")
        else:
            w("typedef enum {")
            for k, v in e["values"].items():
                w(f"    ROBOT_{sn}_{k} = {int(v)},")
            w(f"}} robot_{snake(ename)}_t;\n")

    # structs
    def emit_struct(cname, fields):
        w("typedef struct __attribute__((packed)) {")
        for f in fields:
            ct = SCALARS[f["type"]][0] if not f["is_struct"] else f"robot_{snake(f['type'])}_t"
            arr = f"[{c_count(f['expr'])}]" if f["n"] else ""
            w(f"    {ct} {f['name']}{arr};")
        w(f"}} {cname};")

    for sname, fields in structs.items():
        cn = f"robot_{snake(sname)}_t"
        emit_struct(cn, fields)
        size = sum((f["n"] or 1) * f["esize"] for f in fields)
        w(f'static_assert(sizeof({cn}) == {size}, "{cn} wire size mismatch");\n')
    for pname, p in packets.items():
        cn = f"robot_{snake(pname)}_t"
        sn = snake(pname).upper()
        w(f"#define ROBOT_MSG_{sn} {p['msg_type']}")
        emit_struct(cn, p["fields"])
        w(f'static_assert(sizeof({cn}) == {p["size"]}, "{cn} wire size mismatch (spec/protocol drift)");')
        w(f"#define ROBOT_{sn}_SIZE {p['size']}\n")
    max_payload = max(p["size"] for p in packets.values())
    w(f"#define ROBOT_MAX_PAYLOAD {max_payload}")
    w("#define ROBOT_FRAME_OVERHEAD 7   /* magic2 + type1 + len2 + crc2 */")
    w("#define ROBOT_MAX_FRAME (ROBOT_MAX_PAYLOAD + ROBOT_FRAME_OVERHEAD)\n")

    # joint tables
    w("/* ---- per-joint tables (index == position in robot_spec.yaml `joints:`) ---- */")
    w(jarr_c("uint8_t", "ROBOT_JOINT_ID", [j["id"] for j in J], str))
    w(jarr_c("float", "ROBOT_JOINT_MIN_RAD", [j["min_rad"] for j in J], cfloat))
    w(jarr_c("float", "ROBOT_JOINT_MAX_RAD", [j["max_rad"] for j in J], cfloat))
    w(jarr_c("float", "ROBOT_JOINT_NEUTRAL_RAD", [j["neutral_rad"] for j in J], cfloat))
    w(jarr_c("float", "ROBOT_JOINT_MAX_CURRENT_MA", [j["max_current_ma"] for j in J], cfloat))
    w(jarr_c("float", "ROBOT_JOINT_MAX_VEL_RAD_S", [j["max_vel_rad_s"] for j in J], cfloat))
    w(jarr_c("float", "ROBOT_JOINT_ZERO_OFFSET_RAD", [j["zero_offset_rad"] for j in J], cfloat))
    w(jarr_c("uint8_t", "ROBOT_JOINT_IS_NECK", [1 if j["role"] == "neck" else 0 for j in J], str))
    w(jarr_c("uint8_t", "ROBOT_JOINT_IS_ANKLE", [1 if j["role"] == "ankle" else 0 for j in J], str))
    w("/* 0 = not a gaze joint, 1 = tracks image X (yaw), 2 = tracks image Y (pitch) */")
    w(jarr_c("uint8_t", "ROBOT_JOINT_GAZE_AXIS", [GAZE_AXES.get(j.get("gaze_axis"), 0) for j in J], str))
    w("static const char *const ROBOT_JOINT_NAME[ROBOT_NUM_JOINTS] = { "
      + ", ".join(f'"{j["name"]}"' for j in J) + " };\n")

    # safety / gaze / reflex tunables
    for k, v in spec["safety"].items():
        w(f"#define ROBOT_SAFETY_{k.upper()} {cfloat(v) if isinstance(v, float) else v}")
    for k, v in spec["gaze"].items():
        w(f"#define ROBOT_GAZE_{k.upper()} {cfloat(v) if isinstance(v, float) else v}")
    names = [j["name"] for j in J]
    for rname, r in spec["reflexes"].items():
        rn = rname.upper()
        for k, v in r.items():
            if k == "offsets_rad":
                offs = [float(v.get(n, 0.0)) for n in names]
                w(jarr_c("float", f"ROBOT_REFLEX_{rn}_OFFSETS_RAD", offs, cfloat))
            else:
                w(f"#define ROBOT_REFLEX_{rn}_{k.upper()} {cfloat(v) if isinstance(v, float) else v}")
    w("")

    # CRC + framing
    w(r"""/* ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ---- */
static inline uint16_t robot_crc16_update(uint16_t crc, const uint8_t *data, size_t len) {
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ ROBOT_CRC_POLY) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
static inline uint16_t robot_crc16(const uint8_t *data, size_t len) {
    return robot_crc16_update((uint16_t)ROBOT_CRC_INIT, data, len);
}

/* Payload size this build expects for msg_type, or -1 if the type is unknown. */
static inline int robot_expected_payload_len(uint8_t type) {
    switch (type) {""")
    for pname, p in packets.items():
        w(f"    case ROBOT_MSG_{snake(pname).upper()}: return ROBOT_{snake(pname).upper()}_SIZE;")
    w(r"""    default: return -1;
    }
}

/* Serialise one frame into out[]. Returns bytes written, or 0 if cap too small / bad len. */
static inline size_t robot_frame_encode(uint8_t type, const void *payload, uint16_t len,
                                        uint8_t *out, size_t cap) {
    if (len > ROBOT_MAX_PAYLOAD || cap < (size_t)len + ROBOT_FRAME_OVERHEAD) return 0;
    out[0] = (uint8_t)(ROBOT_MAGIC & 0xFF);
    out[1] = (uint8_t)(ROBOT_MAGIC >> 8);
    out[2] = type;
    out[3] = (uint8_t)(len & 0xFF);
    out[4] = (uint8_t)(len >> 8);
    memcpy(&out[5], payload, len);
    uint16_t crc = robot_crc16(&out[2], (size_t)len + 3);
    out[5 + len] = (uint8_t)(crc & 0xFF);
    out[6 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + ROBOT_FRAME_OVERHEAD;
}

/* Incremental, resynchronising stream parser. Feed bytes one at a time; when
 * robot_parser_feed() returns true, p->type / p->payload / p->len hold a frame
 * whose CRC and length were verified. The frame stays valid until the next feed. */
typedef enum { RP_MAGIC0, RP_MAGIC1, RP_TYPE, RP_LEN0, RP_LEN1, RP_PAYLOAD, RP_CRC0, RP_CRC1 } robot_parser_state_t;
typedef struct {
    robot_parser_state_t st;
    uint8_t type;
    uint16_t len, idx, crc_acc, crc_rx;
    uint8_t payload[ROBOT_MAX_PAYLOAD];
    uint32_t crc_errors, len_errors, frames_ok;
} robot_parser_t;

static inline void robot_parser_init(robot_parser_t *p) { memset(p, 0, sizeof(*p)); p->st = RP_MAGIC0; }

static inline bool robot_parser_feed(robot_parser_t *p, uint8_t b) {
    switch (p->st) {
    case RP_MAGIC0: if (b == (ROBOT_MAGIC & 0xFF)) p->st = RP_MAGIC1; break;
    case RP_MAGIC1:
        if (b == (ROBOT_MAGIC >> 8)) p->st = RP_TYPE;
        else p->st = (b == (ROBOT_MAGIC & 0xFF)) ? RP_MAGIC1 : RP_MAGIC0;
        break;
    case RP_TYPE:
        p->type = b; p->crc_acc = robot_crc16_update((uint16_t)ROBOT_CRC_INIT, &b, 1); p->st = RP_LEN0; break;
    case RP_LEN0:
        p->len = b; p->crc_acc = robot_crc16_update(p->crc_acc, &b, 1); p->st = RP_LEN1; break;
    case RP_LEN1: {
        p->len |= (uint16_t)b << 8; p->crc_acc = robot_crc16_update(p->crc_acc, &b, 1);
        int expect = robot_expected_payload_len(p->type);
        if (expect < 0 || expect != (int)p->len) { p->len_errors++; p->st = RP_MAGIC0; break; }
        p->idx = 0; p->st = RP_PAYLOAD; break;
    }
    case RP_PAYLOAD:
        p->payload[p->idx++] = b; p->crc_acc = robot_crc16_update(p->crc_acc, &b, 1);
        if (p->idx == p->len) p->st = RP_CRC0;
        break;
    case RP_CRC0: p->crc_rx = b; p->st = RP_CRC1; break;
    case RP_CRC1:
        p->crc_rx |= (uint16_t)b << 8; p->st = RP_MAGIC0;
        if (p->crc_rx == p->crc_acc) { p->frames_ok++; return true; }
        p->crc_errors++;
        break;
    }
    return false;
}
""")
    w("#ifdef __cplusplus\n}\n#endif")
    return "\n".join(o) + "\n"


# --------------------------------------------------------------------------- Python emitter
def py_default(f, structs):
    if f["is_struct"]:
        s = f"{f['type']}"
        return f"field(default_factory=lambda: [{s}() for _ in range({f['n']})])" if f["n"] else f"field(default_factory={s})"
    t = f["type"]
    if f["n"]:
        if t == "u8":
            return f"field(default_factory=lambda: bytes({f['n']}))"
        d = f.get("default")
        zero = "0.0" if SCALARS[t][1] in "fd" else "0"
        if d is not None:
            return f"field(default_factory=lambda: {list(d)!r})"
        return f"field(default_factory=lambda: [{zero}] * {f['n']})"
    return "0.0" if SCALARS[t][1] in "fd" else "0"


def py_ann(f):
    if f["is_struct"]:
        return f"list[{f['type']}]" if f["n"] else f["type"]
    if f["n"]:
        return "bytes" if f["type"] == "u8" else "list[float]" if SCALARS[f["type"]][1] in "fd" else "list[int]"
    return "float" if SCALARS[f["type"]][1] in "fd" else "int"


def fmt_chars(fields, structs):
    """Flat struct format (no byte-order prefix) for a field list."""
    s = ""
    for f in fields:
        if f["is_struct"]:
            inner = fmt_chars(structs[f["type"]], structs)
            s += inner * (f["n"] or 1)
        elif f["n"]:
            s += f"{f['n']}s" if f["type"] == "u8" else f"{f['n']}{SCALARS[f['type']][1]}"
        else:
            s += SCALARS[f["type"]][1]
    return s


def values_per_struct(sfields) -> int:
    return sum(1 if not f["n"] else (1 if f["type"] == "u8" else f["n"]) for f in sfields)


def emit_class(cname, fields, structs, fmt, packet=False):
    o = ["@dataclass(slots=True)", f"class {cname}:"]
    for f in fields:
        o.append(f"    {f['name']}: {py_ann(f)} = {py_default(f, structs)}")
    size = sum((f["n"] or 1) * f["esize"] for f in fields)
    o += ["", f"    _STRUCT: ClassVar[struct.Struct] = struct.Struct('<{fmt}')",
          f"    SIZE: ClassVar[int] = {size}", "",
          "    def _flat(self) -> list:", "        v: list = []"]
    for f in fields:
        a = f"self.{f['name']}"
        if f["is_struct"]:
            if f["n"]:
                o.append(f"        for e in {a}: v.extend(e._flat())")
            else:
                o.append(f"        v.extend({a}._flat())")
        elif f["n"]:
            o.append(f"        v.append(bytes({a}))" if f["type"] == "u8" else f"        v.extend({a})")
        else:
            o.append(f"        v.append({a})")
    o += ["        return v", "", "    def pack(self) -> bytes:",
          "        return self._STRUCT.pack(*self._flat())", "",
          "    @classmethod", "    def unpack(cls, data: bytes):",
          "        if len(data) != cls.SIZE:",
          "            raise ValueError(f'{cls.__name__}: expected {cls.SIZE} bytes, got {len(data)}')",
          "        v = cls._STRUCT.unpack(data)", "        i = 0", "        kw = {}"]
    for f in fields:
        nm = f["name"]
        if f["is_struct"]:
            k = values_per_struct(structs[f["type"]])
            if f["n"]:
                o.append(f"        kw['{nm}'] = [{f['type']}.from_flat(v[i + e * {k}: i + (e + 1) * {k}]) for e in range({f['n']})]")
                o.append(f"        i += {k} * {f['n']}")
            else:
                o.append(f"        kw['{nm}'] = {f['type']}.from_flat(v[i:i + {k}]); i += {k}")
        elif f["n"]:
            if f["type"] == "u8":
                o.append(f"        kw['{nm}'] = v[i]; i += 1")
            else:
                o.append(f"        kw['{nm}'] = list(v[i:i + {f['n']}]); i += {f['n']}")
        else:
            o.append(f"        kw['{nm}'] = v[i]; i += 1")
    o.append("        return cls(**kw)")
    if not packet:  # nested structs are rebuilt from a slice of the flat value tuple
        o += ["", "    @classmethod", "    def from_flat(cls, vals):", "        return cls(*vals)"]
    o.append("")
    return "\n".join(o)


def emit_py(spec: dict, proto: dict) -> str:
    J = spec["joints"]
    N = len(J)
    structs, packets = proto["_structs"], proto["_packets"]
    o = []
    w = o.append
    w('"""AUTO-GENERATED by shared/generate_bindings.py -- DO NOT EDIT.')
    w(f"Robot: {spec.get('robot_name', '?')}   joints: {N}   spec hash: {spec_hash()}")
    w('"""')
    w("from __future__ import annotations\n")
    w("import enum\nimport struct\nfrom dataclasses import dataclass, field\nfrom typing import ClassVar, Iterator\n")
    w(f'SPEC_HASH = "{spec_hash()}"')
    w(f"NUM_JOINTS = {N}")
    for k, v in proto["constants"].items():
        w(f"{k} = {int(v)}")
    ctl = spec["control"]
    w(f"STATE_HZ = {int(ctl['state_hz'])}\nACTION_HZ = {int(ctl['action_hz'])}\nACTION_TIMEOUT_MS = {int(ctl['action_timeout_ms'])}")
    w(f"MAGIC = {int(proto['framing']['magic']):#06x}\nCRC_INIT = {int(proto['framing']['crc']['init']):#06x}\nCRC_POLY = {int(proto['framing']['crc']['poly']):#06x}")
    w(f"BUS_DRIVER = {spec['bus']['driver']!r}\nBUS_BAUD = {int(spec['bus']['baud'])}\nROBOT_NAME = {spec.get('robot_name', '')!r}\n")
    w("# ---- per-joint tables (index == position in robot_spec.yaml `joints:`) ----")
    w(f"JOINT_NAMES = {tuple(j['name'] for j in J)!r}")
    w(f"JOINT_IDS = {tuple(j['id'] for j in J)!r}")
    w(f"JOINT_ROLES = {tuple(j['role'] for j in J)!r}")
    for key, nm in (("min_rad", "MIN_RAD"), ("max_rad", "MAX_RAD"), ("neutral_rad", "NEUTRAL_RAD"),
                    ("max_current_ma", "MAX_CURRENT_MA"), ("max_vel_rad_s", "MAX_VEL_RAD_S"),
                    ("zero_offset_rad", "ZERO_OFFSET_RAD")):
        w(f"JOINT_{nm} = {tuple(float(j[key]) for j in J)!r}")
    w(f"NECK_JOINT_INDICES = {tuple(i for i, j in enumerate(J) if j['role'] == 'neck')!r}")
    w(f"SAFETY = {spec['safety']!r}\nGAZE = {spec['gaze']!r}\nREFLEXES = {spec['reflexes']!r}\n")

    for ename, e in proto["enums"].items():
        base = "enum.IntFlag" if e.get("flags") else "enum.IntEnum"
        w(f"class {ename}({base}):")
        for k, v in e["values"].items():
            val = f"1 << {int(v)}" if e.get("flags") else f"{int(v)}"
            w(f"    {k} = {val}")
        w("")

    w("""def crc16(data: bytes, crc: int = CRC_INIT) -> int:
    \"\"\"CRC-16/CCITT-FALSE, bit-identical to robot_crc16() in robot_protocol.h.\"\"\"
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

""")
    w("# Table-driven CRC for speed (built from the same polynomial; checked by self-test below).")
    w("_CRC_TABLE = []\nfor _i in range(256):\n    _c = _i << 8\n    for _ in range(8):\n        _c = ((_c << 1) ^ CRC_POLY) & 0xFFFF if _c & 0x8000 else (_c << 1) & 0xFFFF\n    _CRC_TABLE.append(_c)\n")
    w("""def crc16_fast(data: bytes, crc: int = CRC_INIT) -> int:
    tbl = _CRC_TABLE
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[(crc >> 8) ^ b]
    return crc


assert crc16_fast(b"123456789") == crc16(b"123456789") == 0x29B1  # CCITT-FALSE check value

""")
    for sname, fields in structs.items():
        w(emit_class(sname, fields, structs, fmt_chars(fields, structs)))
    for pname, p in packets.items():
        w(f"MSG_{snake(pname).upper()} = {p['msg_type']}")
        w(emit_class(pname, p["fields"], structs, fmt_chars(p["fields"], structs), packet=True))
    w("PACKET_CLASSES = {" + ", ".join(f"MSG_{snake(n).upper()}: {n}" for n in packets) + "}")
    w("PAYLOAD_SIZES = {t: c.SIZE for t, c in PACKET_CLASSES.items()}")
    w("MAX_PAYLOAD = max(PAYLOAD_SIZES.values())")
    w("FRAME_OVERHEAD = 7\n")
    w('''def encode_frame(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in magic | type | len | payload | crc16."""
    body = struct.pack("<BH", msg_type, len(payload)) + payload
    return struct.pack("<H", MAGIC) + body + struct.pack("<H", crc16_fast(body))


def encode_packet(pkt) -> bytes:
    for t, c in PACKET_CLASSES.items():
        if type(pkt) is c:
            return encode_frame(t, pkt.pack())
    raise TypeError(f"not a packet: {type(pkt)!r}")


class FrameParser:
    """Resynchronising stream parser; mirrors robot_parser_t in the C header."""

    _MAGIC = struct.pack("<H", MAGIC)

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.len_errors = 0
        self.frames_ok = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> Iterator[object]:
        """Add bytes; yield decoded packet dataclasses for every valid frame."""
        buf = self._buf
        buf.extend(data)
        while True:
            i = buf.find(self._MAGIC)
            if i < 0:
                keep = 1 if buf and buf[-1] == self._MAGIC[0] else 0  # possible split magic
                self.dropped_bytes += len(buf) - keep
                del buf[: len(buf) - keep]
                return
            if i:
                self.dropped_bytes += i
                del buf[:i]
            if len(buf) < 5:
                return
            mtype, plen = struct.unpack_from("<BH", buf, 2)
            if PAYLOAD_SIZES.get(mtype) != plen:
                self.len_errors += 1
                del buf[:2]
                continue
            total = plen + FRAME_OVERHEAD
            if len(buf) < total:
                return
            (crc_rx,) = struct.unpack_from("<H", buf, 5 + plen)
            if crc16_fast(bytes(buf[2 : 5 + plen])) != crc_rx:
                self.crc_errors += 1
                del buf[:2]
                continue
            payload = bytes(buf[5 : 5 + plen])
            del buf[:total]
            self.frames_ok += 1
            yield PACKET_CLASSES[mtype].unpack(payload)
''')
    return "\n".join(o) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="verify on-disk outputs are up to date")
    args = ap.parse_args()
    try:
        spec = load_robot()
        proto = load_protocol(len(spec["joints"]))
    except SpecError as e:
        print(f"spec error: {e}", file=sys.stderr)
        return 2
    outputs = {OUT_C: emit_c(spec, proto), OUT_PY: emit_py(spec, proto)}
    bad = False
    for path, text in outputs.items():
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                print(f"STALE: {path.relative_to(ROOT)}  (run `make generate`)", file=sys.stderr)
                bad = True
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8", newline="\n")
            print(f"wrote {path.relative_to(ROOT)}")
    if not args.check:
        init = OUT_PY.parent / "__init__.py"
        if not init.exists():
            init.write_text('"""Generated protocol bindings (see shared/generate_bindings.py)."""\n', encoding="utf-8")
        n = len(spec["joints"])
        sizes = ", ".join(f"{k}={v['size']}B" for k, v in proto["_packets"].items())
        print(f"NUM_JOINTS={n}  {sizes}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
