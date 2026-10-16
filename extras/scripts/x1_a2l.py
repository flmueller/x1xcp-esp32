# PlatformIO post-build step of the x1xcp kit: writes the A2L from the firmware ELF with
# x1-elf2a2l (XCP-Ray), after every build and with "pio run -t a2l".
#
# platformio.ini:
#   extra_scripts = post:scripts/x1_a2l.py
#   custom_x1_variables = a2l/variables.json   ; default
#   custom_x1_a2l = firmware.a2l               ; default: <project name>.a2l
#   custom_x1_elf2a2l = path/to/x1-elf2a2l     ; optional, relative to the project
# The tool is taken from custom_x1_elf2a2l, from $X1_ELF2A2L, from tools/ in the project (copied
# from the kit), or from PATH.
Import("env")  # noqa: F821  (provided by SCons)

import os
import shutil
import subprocess

PROJECT = env.subst("$PROJECT_DIR")  # noqa: F821
VARIABLES = os.path.join(PROJECT, env.GetProjectOption("custom_x1_variables", "a2l/variables.json"))  # noqa: F821
A2L = os.path.join(PROJECT, env.GetProjectOption("custom_x1_a2l", os.path.basename(PROJECT) + ".a2l"))  # noqa: F821


def find_tool():
    exe = ".exe" if os.name == "nt" else ""  # Windows: x1-elf2a2l.exe
    option = env.GetProjectOption("custom_x1_elf2a2l", "")  # noqa: F821
    candidates = [
        os.path.join(PROJECT, option) if option else "",
        os.environ.get("X1_ELF2A2L", ""),
        os.path.join(PROJECT, "tools", "x1-elf2a2l" + exe),
        shutil.which("x1-elf2a2l") or "",
    ]
    return next((c for c in candidates if c and os.path.isfile(c) and os.access(c, os.X_OK)), None)


def generate(source, target, env):
    tool = find_tool()
    if tool is None:
        print("x1-elf2a2l not found (copy tools/ of the kit into the project, or set custom_x1_elf2a2l or X1_ELF2A2L): no A2L generated")
        return
    if not os.path.exists(VARIABLES):
        print("x1_a2l: %s not found (copy a2l/variables.example.json): no A2L generated" % VARIABLES)
        return
    elf = env.subst("$BUILD_DIR/${PROGNAME}.elf")
    result = subprocess.run([tool, elf, "-v", VARIABLES, "-o", A2L])
    if result.returncode != 0:
        env.Exit(1)
    with open(A2L, encoding="utf-8", errors="replace") as a2l:
        text = a2l.read()
    if "/begin MEASUREMENT" not in text and "/begin CHARACTERISTIC" not in text:
        print("x1_a2l: no variable found - does the ELF have debug information? Add build_flags = -g to platformio.ini")


ELF = "$BUILD_DIR/${PROGNAME}.elf"
env.AddPostAction(ELF, generate)  # noqa: F821
if os.path.exists(VARIABLES):
    env.Depends(ELF, VARIABLES)  # noqa: F821  (relinked when the list changes, so the A2L follows)
env.AddCustomTarget("a2l", ELF, generate, title="A2L", description="A2L from the ELF with x1-elf2a2l")  # noqa: F821
