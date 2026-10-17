"""Copy a source-only allowlist into a new directory; never overwrites or deletes files."""
import argparse
from pathlib import Path
import re
import shutil

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('destination', type=Path)
args = parser.parse_args()
dest = args.destination.resolve()
if dest.exists() or root.is_relative_to(dest):
    raise SystemExit('Destination must be a new directory, outside the source ancestry')
dest.mkdir(parents=True)
directories = ['Core', 'Drivers', 'C_board_src', 'cmake', 'tests', 'tools', 'docs', '.github']
for name in directories:
    shutil.copytree(root / name, dest / name,
                    ignore=shutil.ignore_patterns('__pycache__', '框架说明.md'))
for name in ['CMakeLists.txt', 'CMakePresets.json', 'C_project.ioc', 'README.md', '.gitignore',
             'startup_stm32f407xx.s', 'STM32F407xx_FLASH.ld', 'newlib_lock_glue.c', 'stm32_lock.h']:
    shutil.copy2(root / name, dest / name)
pattern = re.compile(r'(?:C:[/\\]Users[/\\]|ghp_[A-Za-z0-9_]{20,}|github_pat_|-----BEGIN PRIVATE KEY-----)')
issues = []
for path in dest.rglob('*'):
    if path.is_file():
        content = path.read_text(encoding='utf-8', errors='replace')
        if pattern.search(content) and path.name not in ('README.md', 'make_submission.py'):
            issues.append(str(path.relative_to(dest)))
if issues:
    raise SystemExit('Review these paths before publication: ' + ', '.join(issues))
print(f'Source package ready: {dest}')
