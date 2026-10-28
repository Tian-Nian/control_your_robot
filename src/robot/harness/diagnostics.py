"""Offline log summaries; never load model messages, images or API configuration."""
import json
from collections import Counter
from pathlib import Path


def summarize_run(directory):
    directory = Path(directory)
    path = directory / 'turns.jsonl'
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError('turn log exceeds 16 MiB')
    records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if records and 'call' in records[0]:
        return summarize_tool_workspace(directory, records)
    statuses, actions, reasons = Counter(), Counter(), Counter()
    close_rounds, checks, heights, depth_rounds = [], [], [], []
    for row in records:
        statuses[str(row.get('status'))] += 1
        action = row.get('action')
        if not isinstance(action, dict):
            continue
        name = str(action.get('action'))
        actions[name] += 1
        if row.get('status') in ('failed', 'rejected'):
            reasons[str(row.get('reason', 'unspecified'))[:300]] += 1
        if name == 'set_gripper' and action.get('gripper') == 0 and row.get('status') in ('succeeded', 'failed'):
            close_rounds.append(row['round'])
        if name == 'point' and 'second' not in action and row.get('status') == 'queried':
            depth_rounds.append(row['round'])
        check = row.get('verification')
        if isinstance(check, dict):
            checks.append({'round': row['round'], 'complete': check.get('complete') is True,
                           'reason': str(check.get('reason', ''))[:200]})
        pose = row.get('actual_feedback', {}).get('tool_pose')
        if isinstance(pose, list) and len(pose) == 7:
            heights.append(float(pose[2]))
    termination = None
    summary_path = directory / 'run_summary.json'
    if summary_path.exists() and summary_path.stat().st_size <= 32_768:
        value = json.loads(summary_path.read_text())
        termination = {key: value[key] for key in ('task_complete', 'exit_reason', 'controller_rounds_started',
                       'last_round', 'stage_reviews', 'stall_replans', 'recovery_replans') if key in value}
    findings = []
    if records and not checks:
        findings.append('no_independent_image_verification')
    if len(close_rounds) > 1:
        findings.append('multiple_close_commands_not_proof_of_multiple_misses')
    if depth_rounds:
        findings.append('single_camera_depth_was_queried')
    return {'run': directory.name, 'turns': len(records),
            'first_round': records[0]['round'] if records else None,
            'last_round': records[-1]['round'] if records else None,
            'statuses': dict(statuses), 'actions': dict(actions), 'failures': dict(reasons),
            'close_rounds': close_rounds, 'image_checks': checks,
            'single_camera_depth_rounds': depth_rounds,
            'tool_z_m': {'first': heights[0], 'last': heights[-1], 'min': min(heights)} if heights else None,
            'findings': findings, 'termination': termination, 'interpretation': 'command/feedback audit only; no grasp success inferred'}


def summarize_tool_workspace(directory, records):
    tools, errors = Counter(), Counter()
    for row in records:
        call = row.get('call') or {}
        tools[str(call.get('tool', 'model_api'))] += 1
        error = row.get('result', {}).get('error')
        if error:
            errors[str(error)[:300]] += 1
    path = directory / 'run_summary.json'
    termination = json.loads(path.read_text()) if path.exists() and path.stat().st_size < 32768 else None
    return dict(run=directory.name, architecture='robowork', turns=len(records), tools=dict(tools),
                tool_errors=dict(errors), termination=termination,
                interpretation='agent tool outcomes; no independent task success inferred')
