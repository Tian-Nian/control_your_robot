"""Bounded live DeepSeek tool-call smoke test; no key or raw model text is logged."""
import json
import os
import time
import urllib.error
import urllib.request

import numpy as np


def tools():
    return [
        {"type": "function", "function": {
            "name": "move_joint_absolute", "description": "Move Piper to six absolute joint angles in radians and verify feedback.",
            "parameters": {"type": "object", "properties": {
                "joint": {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6}},
                "required": ["joint"], "additionalProperties": False}}},
        {"type": "function", "function": {
            "name": "set_gripper", "description": "Set gripper opening [0,1] and verify feedback.",
            "parameters": {"type": "object", "properties": {
                "gripper": {"type": "number", "minimum": 0, "maximum": 1}},
                "required": ["gripper"], "additionalProperties": False}}},
    ]


def check_call(call, expected):
    """Test-specific envelope: refuse any motion outside this fixed experiment."""
    key = "joint" if "joint" in expected else "gripper"
    name = "move_joint_absolute" if key == "joint" else "set_gripper"
    if call.get("type") != "function" or call.get("function", {}).get("name") != name:
        raise ValueError("Unexpected tool or execution order; no action sent")
    args = json.loads(call["function"]["arguments"])
    if not isinstance(args, dict) or set(args) != {key}:
        raise ValueError("Unexpected tool arguments; no action sent")
    actual = np.asarray(args[key], dtype=float)
    target = np.asarray(expected[key], dtype=float)
    if actual.shape != target.shape or not np.isfinite(actual).all() or not np.allclose(actual, target, atol=1e-8, rtol=0):
        raise ValueError("Model target outside this test task; no action sent")
    return args


def run_deepseek(executor, requests, model):
    key = os.environ.get("DEEPSEEK_API_KEY", "")
    if not key:
        raise RuntimeError("Export DEEPSEEK_API_KEY before running this example")
    initial = executor.session.read_raw(executor.component)["data"]
    messages = [
        {"role": "system", "content": "You control an isolated Piper simulator via tools. Call exactly one tool per reply, wait for its verified result, then continue. Never claim success before feedback. Do not call other tools. After all three skills succeed, give a short completion message without a tool call."},
        {"role": "user", "content": json.dumps({
            "capabilities": executor.capabilities(), "initial_joint_rad": np.asarray(initial["joint"]).tolist(),
            "task": "First increase only joint 2 by 0.05 rad from the initial pose. Then set gripper to 0.5. Finally return all joints to the initial pose, retaining gripper opening. Use absolute targets. Execute each step only after the preceding step succeeds."
        })},
    ]
    outcomes, usage = [], []
    for round_index in range(4):
        payload = {"model": model, "messages": messages, "tools": tools(),
                   "thinking": {"type": "disabled"}, "max_tokens": 600, "temperature": 0}
        request = urllib.request.Request("https://api.deepseek.com/chat/completions",
            data=json.dumps(payload).encode(), headers={"Authorization": "Bearer " + key,
            "Content-Type": "application/json"}, method="POST")
        start = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                body = json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"DeepSeek HTTP {exc.code}; response body hidden") from None
        except Exception as exc:
            raise RuntimeError(f"DeepSeek transport failure ({type(exc).__name__}); details hidden") from None
        elapsed = time.monotonic() - start
        message = body["choices"][0]["message"]
        calls = message.get("tool_calls") or []
        usage.append(body.get("usage", {}))
        print(f"DeepSeek round {round_index + 1}: {elapsed:.2f}s, tool_calls={len(calls)}", flush=True)
        if round_index == 3:
            if calls or len(outcomes) != 3 or not message.get("content"):
                raise RuntimeError("Model did not finish the bounded task")
            break
        if len(calls) != 1:
            raise RuntimeError("Expected exactly one tool call; no action sent")
        call = calls[0]
        args = check_call(call, requests[round_index])
        result = executor.execute(requests[round_index]["request_id"], **args)
        outcomes.append(result)
        print(f"  {call['function']['name']}: {result['status']}", flush=True)
        if result["status"] != "succeeded":
            raise RuntimeError("Harness verification failed; task stopped")
        messages.append({"role": "assistant", "content": message.get("content"), "tool_calls": calls})
        messages.append({"role": "tool", "tool_call_id": call["id"], "content": json.dumps(result)})
    total_tokens = sum(int(item.get("total_tokens", 0)) for item in usage)
    print(f"DeepSeek model={model}; API calls=4; total_tokens={total_tokens}", flush=True)
    return outcomes
