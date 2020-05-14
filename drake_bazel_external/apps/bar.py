import sys
import builtins
from os.path import realpath, dirname

import drake as m
runfiles = dirname(dirname(m.__file__))
source_tree = dirname(dirname(realpath(__file__)))

def print(x=""):
    x = x.replace(runfiles, "{runfiles}")
    x = x.replace(source_tree, "{source_tree}")
    return builtins.print(x)

# Print sys.path
print("path:")
for p in sys.path:
    print(f"  {p}")
print()

# Import module, show path, then repro error.
import common as m
print(f"common: {m.__file__}")
print()
from common import foo

"""
To get clean output in `bash`:

    bazel build //apps:bar
    alias bash-isolate='env -i HOME=$HOME DISPLAY=$DISPLAY SHELL=$SHELL TERM=$TERM USER=$USER PATH=/usr/local/bin:/usr/bin:/bin bash --norc'
    bash-isolate -c bazel-bin/apps/bar

Example output:

```
    path:
      {source_tree}/apps
      {runfiles}
      {runfiles}/drake/bindings
      {runfiles}/lcmtypes_bot2_core/lcmtypes
      {runfiles}/lcmtypes_bot2_core
      {runfiles}/lcmtypes_robotlocomotion/lcmtypes
      {runfiles}/lcmtypes_robotlocomotion
      {runfiles}/meshcat_python/src
      {runfiles}/spdlog
      {runfiles}/meshcat_python
      {runfiles}/lcm
      {runfiles}/ignition_math
      {runfiles}/drake
      {runfiles}/drake_external_examples
      /usr/lib/python36.zip
      /usr/lib/python3.6
      /usr/lib/python3.6/lib-dynload
      /usr/lib/python3/dist-packages

    common: {runfiles}/drake/common/__init__.py
```

See relevant Bazel issue:
https://github.com/bazelbuild/bazel/issues/7653

See relevant Drake issue:
https://github.com/RobotLocomotion/drake/issues/7871

"""
