#!/usr/bin/env python3
"""Render a service unit to stdout. Never installs or starts anything."""
import argparse
from pathlib import Path


def unit_escape(value: str) -> str:
    if any(char in value for char in ('\n', '\r', '\x00')):
        raise ValueError('Unit values cannot contain newlines or NUL')
    return value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')


def render(project: Path, bridge_user: str, env_file: Path | None = None) -> str:
    template = Path(__file__).with_name('wdblh.service').read_text()
    absolute = str(project.resolve())
    # WorkingDirectory= takes the whole path verbatim, unlike ExecStart's argv.
    # Validate it using the same control-character rules; escape only specifiers.
    unit_escape(absolute)
    config_line = '' if env_file is None else f'Environment="WDBLH_ENV_FILE={unit_escape(str(env_file.resolve()))}"'
    return template.replace('@ENVIRONMENT_FILE@', config_line).replace('@WORKING_DIRECTORY@', absolute.replace('%', '%%')).replace('@PROJECT@', unit_escape(absolute)).replace(
        '@BRIDGE_USER@', unit_escape(bridge_user))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--user', required=True)
    parser.add_argument('--env-file', type=Path)
    args = parser.parse_args()
    print(render(args.project, args.user, args.env_file), end='')
