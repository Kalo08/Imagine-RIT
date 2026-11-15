# Thin wrapper; every target is a plain `python` command so Windows users without
# make can run the same line by hand (see README).
PY      ?= python
LINK    ?= tcp://127.0.0.1:5555
export PYTHONPATH := $(CURDIR)/brain_pi/src

.PHONY: help generate check-generated install bench-test sim run-sim jog info validate-policy calibrate-sim firmware-build firmware-flash clean

help:
	@echo "generate         regenerate C + Python bindings from shared/*.yaml"
	@echo "check-generated  fail if generated files are stale"
	@echo "install          uv sync brain_pi (add EXTRA=--extra onnx for onnxruntime)"
	@echo "bench-test       sim_bridge + brain_pi neutral_hold, 200 Hz state / 50 Hz action check"
	@echo "sim              run sim_bridge alone (tcp://127.0.0.1:5555)"
	@echo "run-sim          brain_pi against a running sim (LINK=...)"
	@echo "jog              interactive manual jog (LINK=...)"
	@echo "validate-policy  check brain_pi/policies/ bundle against robot_spec.yaml"
	@echo "firmware-build   idf.py build for esp32p4"

generate:
	$(PY) shared/generate_bindings.py

check-generated:
	$(PY) shared/generate_bindings.py --check

install:
	cd brain_pi && uv sync $(EXTRA)

bench-test: check-generated
	$(PY) tools/bench_test.py

sim: check-generated
	$(PY) tools/sim_bridge.py

run-sim:
	$(PY) -m cli run --mode neutral_hold --link $(LINK)

jog:
	$(PY) -m cli jog --link $(LINK)

info:
	$(PY) -m cli info

validate-policy:
	$(PY) -m cli validate-policy brain_pi/policies

calibrate-sim:
	$(PY) tools/servo_zero_calibrator.py --link $(LINK) --yes --dry-run

firmware-build: check-generated
	cd firmware_esp32p4 && idf.py set-target esp32p4 && idf.py build

firmware-flash:
	cd firmware_esp32p4 && idf.py flash monitor

clean:
	cd firmware_esp32p4 && idf.py fullclean || true
