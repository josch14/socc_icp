import yaml
from deepmerge import always_merger  # pip install deepmerge

CONFIG_DEFAULTS = "configs/config_defaults.yaml"


def load_config_dict(
    path_defaults: str | None = None, path_override: str | None = None
) -> dict:
    if path_defaults is None:
        path_defaults = CONFIG_DEFAULTS

    with open(path_defaults) as f:
        base = yaml.safe_load(f)

    if path_override:
        with open(path_override) as f:
            override = yaml.safe_load(f)
        # rule: for every key in both dictionaries, the value from override replaces the value in base
        # and nested dictionaries are merged recursively
        config = always_merger.merge(base, override)
        return config

    return base
