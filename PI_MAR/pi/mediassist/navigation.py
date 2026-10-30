"""Endpoint selection. Only failed planning may select another group member.

An aborted NavigateToPose is deliberately NOT treated as proof of a blocked goal.
"""
from .contracts import StepError


def candidates(params, config):
    if ("endpoint_id" in params) == ("endpoint_group_id" in params):
        raise StepError("EXACTLY_ONE_TARGET_REQUIRED")
    if "endpoint_id" in params:
        ids = [params["endpoint_id"]]
    else:
        group = config.get("endpoint_groups", {}).get(params["endpoint_group_id"])
        if not group or group["map_id"] != params["map_id"]:
            raise StepError("ENDPOINT_GROUP_NOT_FOUND")
        if params["group_revision"] != group["revision"]:
            raise StepError("GROUP_REVISION_MISMATCH")
        ids = params["candidate_endpoint_ids"]
        if not ids or len(ids) != len(set(ids)) or not set(ids) <= set(group["endpoint_ids"]):
            raise StepError("INVALID_GROUP_MEMBERS")
        if len(ids) > config["navigation"]["max_candidates"]:
            raise StepError("CANDIDATE_LIMIT")
    for endpoint_id in ids:
        point = config["endpoints"].get(endpoint_id)
        if not point or point["map_id"] != params["map_id"]:
            raise StepError("ENDPOINT_NOT_FOUND", endpoint_id)
    return list(ids)


async def select_and_move(params, config, plan, move, report):
    ids = candidates(params, config)
    attempts = []
    for endpoint_id in ids:
        try:
            await plan(endpoint_id)
        except StepError as exc:
            attempts.append({"endpoint_id": endpoint_id, "phase": "PLAN", "outcome": exc.code})
            report(attempts[-1])
            if exc.code not in ("PATH_UNAVAILABLE", "ENDPOINT_FOOTPRINT_NOT_CLEAR") or "endpoint_group_id" not in params:
                raise
            continue
        report({"endpoint_id": endpoint_id, "phase": "PLAN", "outcome": "PATH_AVAILABLE"})
        # No catch/retry here: cancellation, timeout, sensor loss and NAV_ABORTED stop the step.
        try:
            result = await move(endpoint_id)
        except StepError as exc:
            report({"endpoint_id": endpoint_id, "phase": "NAVIGATE", "outcome": exc.code})
            raise
        attempts.append({"endpoint_id": endpoint_id, "phase": "NAVIGATE", "outcome": "REACHED"})
        report(attempts[-1])
        return {**result, "endpoint_group_id": params.get("endpoint_group_id"), "attempts": attempts}
    raise StepError("NO_REACHABLE_ENDPOINT")
