"""Contracts, bounded expression evaluator, and workflow validation. No eval/exec."""
import copy
import hashlib
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker


class StepError(Exception):
    def __init__(self, code, message="", retryable=False):
        super().__init__(message or code)
        self.code, self.retryable = code, retryable


def parse_timestamp(value):
    """RFC3339 UTC/offset timestamp, including Java nanoseconds, on Python 3.10.

    datetime only stores microseconds. Truncate sub-microsecond precision;
    never round an expired motion command into the future.
    """
    if not isinstance(value, str):
        raise ValueError('Timestamp must be a string')
    match = re.fullmatch(r'(\d{4}-\d{2}-\d{2})[Tt](\d{2}:\d{2}:\d{2})(?:\.(\d{1,9}))?([Zz]|[+-]\d{2}:\d{2})', value)
    if not match:
        raise ValueError('Timestamp must be RFC3339 with a timezone')
    date, clock, fraction, offset = match.groups()
    suffix = '.' + fraction[:6].ljust(6, '0') if fraction else ''
    offset = '+00:00' if offset in ('Z', 'z') else offset
    return datetime.fromisoformat(date + 'T' + clock + suffix + offset).astimezone(timezone.utc)


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode("utf-8")).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def validate(value, schema):
    canonical(value)  # Reject NaN/Infinity, including values obtained locally.
    errors = sorted(Draft202012Validator(schema, format_checker=FormatChecker()).iter_errors(value), key=lambda e: str(e.path))
    if errors:
        err = errors[0]
        raise StepError("SCHEMA_INVALID", f"{list(err.path)}: {err.message}")


def pointer(root, path):
    if path == "":
        return copy.deepcopy(root)
    if not isinstance(path, str) or not path.startswith("/"):
        raise StepError("INVALID_PATH", str(path))
    value = root
    try:
        for raw in path[1:].split("/"):
            key = raw.replace("~1", "/").replace("~0", "~")
            value = value[int(key)] if isinstance(value, list) else value[key]
    except (KeyError, IndexError, ValueError, TypeError):
        raise StepError("MISSING_VALUE", path) from None
    return copy.deepcopy(value)


def resolve(template, context, depth=0):
    if depth > 32:
        raise StepError("BINDING_DEPTH")
    if isinstance(template, dict):
        if set(template) == {"$ref"}:
            return pointer(context, template["$ref"])
        return {k: resolve(v, context, depth + 1) for k, v in template.items()}
    if isinstance(template, list):
        return [resolve(v, context, depth + 1) for v in template]
    return copy.deepcopy(template)


def evaluate(expr, context, budget=None):
    """SAFE_EXPR_V1: var, eq/ne, gt/ge/lt/le, and/or/not, add. Strict types."""
    budget = [128] if budget is None else budget
    budget[0] -= 1
    if budget[0] < 0:
        raise StepError("EXPRESSION_LIMIT")
    if not isinstance(expr, dict):
        if expr is None or isinstance(expr, (bool, int, float, str)):
            return expr
        raise StepError("INVALID_EXPRESSION")
    if set(expr) == {"var"}:
        return pointer(context, expr["var"])
    if set(expr) != {"op", "args"} or not isinstance(expr["args"], list):
        raise StepError("INVALID_EXPRESSION")
    op = expr["op"]
    args = [evaluate(x, context, budget) for x in expr["args"]]
    if op in ("and", "or", "not"):
        if not args or any(type(x) is not bool for x in args) or (op == "not" and len(args) != 1):
            raise StepError("TYPE_MISMATCH")
        return not args[0] if op == "not" else (all(args) if op == "and" else any(args))
    if len(args) != 2:
        raise StepError("INVALID_EXPRESSION")
    a, b = args
    if op in ("eq", "ne"):
        equal = type(a) is type(b) and a == b
        return equal if op == "eq" else not equal
    if type(a) not in (int, float) or type(b) not in (int, float):
        raise StepError("TYPE_MISMATCH")
    if not math.isfinite(a) or not math.isfinite(b):
        raise StepError("TYPE_MISMATCH")
    functions = {"gt": lambda: a > b, "ge": lambda: a >= b, "lt": lambda: a < b,
                 "le": lambda: a <= b, "add": lambda: a + b}
    if op not in functions:
        raise StepError("INVALID_OPERATOR", op)
    return functions[op]()


class Catalog:
    def __init__(self, root):
        self.root = Path(root)
        self.types = read_json(self.root / "schemas/step_types.json")
        self.workflow_schema = read_json(self.root / "schemas/workflow.schema.json")
        self.workflows = {}
        for path in sorted((self.root / "workflows").glob("*.json")):
            wf = read_json(path)
            self.check_workflow(wf)
            self.workflows[(wf["code"], wf["version"])] = wf

    def check_workflow(self, wf):
        validate(wf, self.workflow_schema)
        codes = [s["code"] for s in wf["steps"]]
        if len(set(codes)) != len(codes) or wf["start"] not in codes:
            raise StepError("INVALID_GRAPH")
        tasks = [t["code"] for t in wf["tasks"]]
        if len(set(tasks)) != len(tasks): raise StepError("DUPLICATE_TASK_CODE")
        for step in wf["steps"]:
            if step["task_code"] not in tasks: raise StepError("TASK_NOT_FOUND")
            if step["type"] not in self.types:
                raise StepError("UNKNOWN_STEP_TYPE")
            seen = set()
            for tr in step["transitions"]:
                key = (tr["on"], tr["priority"])
                if key in seen or ("next" in tr and tr["next"] not in codes):
                    raise StepError("INVALID_TRANSITION", step["code"])
                seen.add(key)
            outcomes = {t["on"] for t in step["transitions"] if "when" not in t and "error_code" not in t}
            if not {"FAILURE", "TIMEOUT"} <= outcomes:
                raise StepError("MISSING_FALLBACK", step["code"])
            def inspect(value):
                if isinstance(value, dict):
                    if set(value) == {"$ref"}:
                        parts = value["$ref"].split("/")
                        if len(parts) < 2 or parts[1] not in ("input", "outputs", "vars", "system"):
                            raise StepError("INVALID_BINDING")
                        if parts[1] == "input" and len(parts) > 2 and parts[2] not in wf["input_schema"]["properties"]:
                            raise StepError("UNDECLARED_INPUT", parts[2])
                        if parts[1] == "outputs" and (len(parts) < 3 or parts[2] not in codes):
                            raise StepError("UNDECLARED_OUTPUT", value["$ref"])
                    else:
                        for v in value.values(): inspect(v)
                elif isinstance(value, list):
                    for v in value: inspect(v)
            inspect(step["params"])
        reached, pending = set(), [wf["start"]]
        while pending:
            code = pending.pop()
            if code in reached: continue
            reached.add(code)
            step = next(s for s in wf["steps"] if s["code"] == code)
            pending.extend(t["next"] for t in step["transitions"] if "next" in t)
        if reached != set(codes):
            raise StepError("UNREACHABLE_STEPS", str(set(codes) - reached))

    def get(self, code, version):
        try:
            return self.workflows[(code, version)]
        except KeyError:
            raise StepError("WORKFLOW_NOT_FOUND", f"{code}@{version}") from None

    def check_params(self, kind, params):
        validate(params, self.types[kind]["input_schema"])

    def check_output(self, kind, result):
        validate(result, self.types[kind]["output_schema"])
