#!/usr/bin/env python3
"""Exercise an AIDEM policy on QNN DSP without opening robot devices."""

import argparse
import json
import time
from pathlib import Path

import aidlite
import numpy as np


def check(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation} failed: {code}")


def tensor_info(graphs):
    return [[{
        "name": t.name,
        "shape": list(t.shape),
        "elements": int(t.element_count),
        "type": str(t.element_type),
    } for t in graph] for graph in graphs]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--runs", type=int, default=200)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    model_path = args.model.resolve(strict=True)
    report = {"model": str(model_path), "backend": "QNN248/DSP", "ok": False}
    interpreter = None
    try:
        aidlite.set_log_level(aidlite.LogLevel.WARNING)
        aidlite.log_to_stderr()
        report["aidlite_version"] = aidlite.get_library_version()
        model = aidlite.Model.create_instance(str(model_path))
        config = aidlite.Config.create_instance()
        if model is None or config is None:
            raise RuntimeError("Could not create model/config")
        config.framework_type = aidlite.FrameworkType.TYPE_QNN248
        config.accelerate_type = aidlite.AccelerateType.TYPE_DSP
        interpreter = aidlite.InterpreterBuilder.build_interpreter_from_model_and_config(model, config)
        if interpreter is None:
            raise RuntimeError("Could not build interpreter")
        start = time.perf_counter()
        check(interpreter.init(), "init")
        check(interpreter.load_model(), "load_model")
        report["load_ms"] = (time.perf_counter() - start) * 1000
        inputs = interpreter.get_input_tensor_info()
        outputs = interpreter.get_output_tensor_info()
        report["inputs"] = tensor_info(inputs)
        report["outputs"] = tensor_info(outputs)
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if len(inputs) != 1 or len(inputs[0]) != 1 or len(outputs) != 1 or len(outputs[0]) != 1:
            raise RuntimeError("Expected one graph, one input and one output")
        size = int(inputs[0][0].element_count)
        if size not in (270, 276) or int(outputs[0][0].element_count) != 12:
            raise RuntimeError("Expected 270/276 input elements and 12 actions")
        shape = tuple(inputs[0][0].shape)

        def forward(data):
            # Non-native mode lets AidLite convert float32 to the native tensor type.
            check(interpreter.set_input_tensor(0, np.ascontiguousarray(data, dtype=np.float32)), "set_input_tensor")
            check(interpreter.invoke(), "invoke")
            result = interpreter.get_output_tensor(0, aidlite.DataType.TYPE_FLOAT32).copy()
            if result.size != 12 or not np.isfinite(result).all():
                raise RuntimeError("Invalid action shape or NaN/Inf")
            return result

        nominal = np.zeros(size, dtype=np.float32)
        frames = nominal.reshape(6, size // 6)
        gravity_z = 9 if size == 276 else 8
        frames[:, gravity_z] = -1
        if size == 276:
            frames[:, 3] = 0.35 * 2.0
        rng = np.random.default_rng(30)
        samples = {"zeros": np.zeros(size, dtype=np.float32), "nominal": nominal,
                   "perturbed": nominal + rng.normal(0, 0.05, size).astype(np.float32)}
        report["sample_actions"] = {name: forward(x.reshape(shape)).tolist() for name, x in samples.items()}
        for _ in range(20):
            forward(nominal.reshape(shape))
        timings = []
        maximum = 0.0
        for _ in range(args.runs):
            data = nominal + rng.normal(0, 0.05, size).astype(np.float32)
            start = time.perf_counter()
            result = forward(data.reshape(shape))
            timings.append((time.perf_counter() - start) * 1000)
            maximum = max(maximum, float(np.max(np.abs(result))))
        report["runs"] = args.runs
        report["end_to_end_ms"] = {"mean": float(np.mean(timings)), "median": float(np.median(timings)),
                                   "p95": float(np.percentile(timings, 95)), "max": float(np.max(timings))}
        report["max_abs_action"] = maximum
        report["ok"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        if interpreter is not None:
            try:
                interpreter.destroy()
            except Exception as error:
                report["cleanup_error"] = str(error)
        if args.report:
            args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
