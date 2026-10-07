"""PlatformIO pre-script: fails the build when a source file is not clang-formatted.

Wired into [env:native], so `pio test -e native` checks formatting first. Fix with
`pio run -e native -t format` (see platformio.ini) or by running clang-format -i on the files.
"""
import os
import shutil
import subprocess
import sys

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

PROJECT_DIR = env.subst("$PROJECT_DIR")  # noqa: F821
DIRS = ("src", "examples", "test")
EXTENSIONS = (".h", ".cpp", ".ino")


def find_clang_format():
    try:
        import clang_format

        return clang_format.get_executable("clang-format")
    except ImportError:
        return shutil.which("clang-format")


def source_files():
    for top in DIRS:
        for folder, _, names in os.walk(os.path.join(PROJECT_DIR, top)):
            for name in names:
                if name.endswith(EXTENSIONS):
                    yield os.path.join(folder, name)


def fail(message):
    sys.stderr.write("check_format: " + message + "\n")
    env.Exit(1)  # noqa: F821


exe = find_clang_format()
if exe is None:
    fail("clang-format not found. Install it with: pip install clang-format")

files = list(source_files())
result = subprocess.run(
    [exe, "--dry-run", "--Werror", "--style=file"] + files,
    cwd=PROJECT_DIR,
    capture_output=True,
    text=True,
)
if result.returncode != 0:
    sys.stderr.write(result.stderr)
    fail("formatting differs from .clang-format. Run clang-format -i on the files listed above.")
print("check_format: %d files OK" % len(files))
