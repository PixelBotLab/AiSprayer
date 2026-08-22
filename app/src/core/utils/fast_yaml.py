import yaml

try:
    from yaml import CSafeLoader as FastYamlLoader, CDumper as FastYamlDumper
except ImportError:
    from yaml import SafeLoader as FastYamlLoader, Dumper as FastYamlDumper


class FlowList(list):
    """Marker class for lists that should be dumped in compact JSON/Flow style [a, b, c]."""
    pass


def _flow_list_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)


# Register representer for FlowList on standard and fast dumpers
yaml.add_representer(FlowList, _flow_list_representer)
try:
    FastYamlDumper.add_representer(FlowList, _flow_list_representer)
except Exception:
    pass


def fast_yaml_load(stream_or_str):
    """Ultra-fast YAML load using C-libyaml parser when available."""
    if stream_or_str is None:
        return {}
    if isinstance(stream_or_str, str):
        return yaml.load(stream_or_str, Loader=FastYamlLoader) or {}
    return yaml.load(stream_or_str, Loader=FastYamlLoader) or {}


def _compact_data_structures(data):
    """
    Recursively converts coordinate vectors, joint angles, and trajectory arrays
    into FlowList so they are emitted as compact single-line flow sequences [a, b, c].
    """
    if isinstance(data, dict):
        return {k: _compact_data_structures(v) for k, v in data.items()}
    elif isinstance(data, list):
        if len(data) > 0 and all(isinstance(x, (int, float, str)) and not isinstance(x, bool) for x in data):
            # Short primitives list like [1, 2, 3] or joint velocity list
            return FlowList(data)
        elif len(data) > 0 and all(isinstance(x, (list, tuple)) for x in data):
            # 2D arrays like trajectory_q [[...], [...]] or dense_surface_points_base_mm
            return [_compact_data_structures(item) for item in data]
        else:
            return [_compact_data_structures(x) for x in data]
    return data


def fast_yaml_dump(data, stream=None, **kwargs):
    """
    Dumps Python dict to compact, clean YAML with flow vectors.
    """
    compacted = _compact_data_structures(data)
    kwargs.setdefault('allow_unicode', True)
    kwargs.setdefault('default_flow_style', False)
    kwargs.setdefault('sort_keys', False)
    return yaml.dump(compacted, stream=stream, **kwargs)
